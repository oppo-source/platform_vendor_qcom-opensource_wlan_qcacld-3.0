#include "wlan_2g_ht40.h"
#include <linux/oplus_kevent.h>
#include "../../core/hdd/src/wlan_hdd_stats.h"
#include "../../../qca-wifi-host-cmn/qdf/linux/src/i_qdf_trace.h"

smart_bw_rus_cfg_t g_acs_threshold_cfg =
	{0, QDF_TRACE_LEVEL_DEBUG, 85, 35100, 8, 100, 3, 5, 2};
#define smbw_err(params...)   QDF_TRACE_ERROR(QDF_MODULE_ID_HDD, params)
#define smbw_debug(params...) \
	g_acs_threshold_cfg.debug_level == QDF_TRACE_LEVEL_DEBUG ? \
	QDF_TRACE_DEBUG(QDF_MODULE_ID_HDD, params) : \
	QDF_TRACE_ERROR(QDF_MODULE_ID_HDD, params)

#define CHAN_BUSY_THRESHOLD 41	/*41 means 85% chan time used, 48 means 100% used*/
#define STATS_SAMPLE_NUM    4
#define FULL_WEIGHT	(26664 * 2)
#define AGE_30S		30000


struct smart_bw_mgr smart_bw_mgr_g;
struct scan_res_stats scan_res_stats_g;

struct scan_res_stats *g_res_stats = NULL;
manage_bss_list_t   g_manage_bss_list;
last_record_t       g_last_connect_record;


#if 1
extern bool get_smart_bw_rom_update(int payload[], int len);
#endif

#define FEATURE_DISABLE (smart_bw_mgr_g.func_enable != 1)
#define ACS_WEIGHT_AMOUNT_LOCAL    240

#define ABS(_x) ((int)_x > 0 ? (int)_x : -(int)_x)

#define ACS_WEIGHT_AMOUNT_CONFIG(weights) \
	(((weights) & 0xf) + \
	(((weights) & 0xf0) >> 4) + \
	(((weights) & 0xf00) >> 8) + \
	(((weights) & 0xf000) >> 12) + \
	(((weights) & 0xf0000) >> 16) + \
	(((weights) & 0xf00000) >> 20))

#define ACS_WEIGHT_COMPUTE(weights, weight, factor, base) \
	(((((((((weight) << 4) * ACS_WEIGHT_AMOUNT_LOCAL * (factor)) + \
	(ACS_WEIGHT_AMOUNT_CONFIG((weights)) >> 1)) / \
	ACS_WEIGHT_AMOUNT_CONFIG((weights))) + \
	((base) >> 1)) / (base)) + 8) >> 4)

#define ACS_WEIGHT_CFG_TO_LOCAL(weights, weight) \
	(((((((weight) << 4) * ACS_WEIGHT_AMOUNT_LOCAL) + \
	(ACS_WEIGHT_AMOUNT_CONFIG((weights)) >> 1)) / \
	ACS_WEIGHT_AMOUNT_CONFIG((weights))) + 8) >> 4)

#define ACS_WEIGHT_RSSI_CFG(weights) \
	((weights) & 0xf)

#define ACS_WEIGHT_COUNT_CFG(weights) \
	(((weights) & 0xf0) >> 4)

#define ACS_WEIGHT_NOISE_FLOOR_CFG(weights) \
	(((weights) & 0xf00) >> 8)

#define ACS_WEIGHT_CHANNEL_FREE_CFG(weights) \
	(((weights) & 0xf000) >> 12)

#define ACS_WEIGHT_TX_POWER_RANGE_CFG(weights) \
	(((weights) & 0xf0000) >> 16)

#define ACS_WEIGHT_TX_POWER_THROUGHPUT_CFG(weights) \
	(((weights) & 0xf00000) >> 20)

static unsigned char freq_to_chan_2g(unsigned int freq)
{
	unsigned char chan = 0;

	if (freq > WLAN_24_GHZ_BASE_FREQ && freq < WLAN_CHAN_14_FREQ)
		chan = ((freq - WLAN_24_GHZ_BASE_FREQ) /
			WLAN_CHAN_SPACING_5MHZ);
	else if (freq == WLAN_CHAN_14_FREQ)
		chan = WLAN_24_GHZ_CHANNEL_14;

	return chan;
}

static unsigned int
moving_avg(unsigned int cur_data, unsigned int parent_data, unsigned int beta)
{
    unsigned int avg_data = 0;
    
    if (parent_data > 0) {
        avg_data =
        (parent_data * beta + cur_data * (16 - beta)) / 16;
    }  else {
        avg_data = cur_data;
    }

    return avg_data;
}

static int get_chan_free_weight(struct smart_bw_mgr *bw_mgr)
{
	int chan_free_weight = 0, i = 0;
	unsigned char af_chan_start = 0, af_chan_end = 0;

	af_chan_start = bw_mgr->ht2g_cur_bss.af_chan_start;
	af_chan_end = bw_mgr->ht2g_cur_bss.af_chan_end;
	for (i = af_chan_start - 1; i < af_chan_end; i++) {
		if (smart_bw_mgr_g.chan_free_weight[i] > chan_free_weight) {
			chan_free_weight = smart_bw_mgr_g.chan_free_weight[i];
		}
	}

	return chan_free_weight;
}

static int check_ht40_chan_busy(struct smart_bw_mgr *bw_mgr)
{
	int chan_free_weight = 0, ht40_chan_busy = 0;

	chan_free_weight = get_chan_free_weight(bw_mgr);

	if (chan_free_weight >= CHAN_BUSY_THRESHOLD)
		ht40_chan_busy = 1;
	g_last_connect_record.ht40_chan_busy = ht40_chan_busy;

	return ht40_chan_busy;
}

static void handle_exist_ht40bss(struct smart_bw_mgr *bw_mgr,
	unsigned char pri_chan, unsigned char sec_chan)
{
	if (bw_mgr->res_stats->ht40_chan_bssinfo[pri_chan].nExt20 > 0 ||
		bw_mgr->res_stats->ht40_chan_bssinfo[sec_chan].nCtrl > 0) {
		bw_mgr->bw_decision.exist_ht40_conflict = 1;
	} else {
		bw_mgr->bw_decision.exist_ht40_conflict = 0;
	}
	g_last_connect_record.exist_ht40_conflict =
		bw_mgr->bw_decision.exist_ht40_conflict;
	if (bw_mgr->bw_decision.exist_ht40_conflict) {
		smbw_err("smbw--ht40_conflict:%d,nCtrl:%d,nExt20:%d,pri_chan:%d,sec_chan:%d@%s, %d",
		bw_mgr->bw_decision.exist_ht40_conflict,
		bw_mgr->res_stats->ht40_chan_bssinfo[sec_chan].nCtrl,
		bw_mgr->res_stats->ht40_chan_bssinfo[pri_chan].nExt20,
		pri_chan, sec_chan,
		__func__, __LINE__);
	}
}

static unsigned char get_total_num_intol(struct smart_bw_mgr *bw_mgr,
	unsigned char chan_start,
	unsigned char chan_end)
{
	unsigned char chan_idx = 0;
	unsigned char total_num_intol = 0;

	for (chan_idx = chan_start; chan_idx <= chan_end; chan_idx++) {
		total_num_intol +=
			bw_mgr->res_stats->ht40_intolerant[chan_idx];
	}
	smbw_err("smbw--total_num_intol is %d, @%s, %d",
		total_num_intol, __func__, __LINE__);
	return total_num_intol;
}

static unsigned int get_total_bg_cnt(struct smart_bw_mgr *bw_mgr,
	unsigned char chan_start,
	unsigned char chan_end)
{
	unsigned char chan_idx = 0;
	unsigned int total_bg_cnt = 0;

	for (chan_idx = chan_start; chan_idx <= chan_end; chan_idx++) {
		total_bg_cnt += bw_mgr->res_stats->bg_bss_num[chan_idx];
	}
	smbw_debug("smbw--total_bg_cnt is %d, @%s, %d",
		total_bg_cnt, __func__, __LINE__);
	return total_bg_cnt;
}

static const char *logid2str(wcn_key_log_id log_id)
{
#define ID2Str(x) case x: return #x
	switch(log_id) {
	ID2Str(scan_band_ratio);
	ID2Str(connect_band_ratio);
	default:
		return "SMBW_UNKNOW";
	}
}

static unsigned char cal_avg_weight(weight_id wid, unsigned int weight)
{
	unsigned char *parent_weight = NULL;
	unsigned char avg_weight = 0;
	struct ht2g_bss_info *cur_bss_info = NULL;

	cur_bss_info = &smart_bw_mgr_g.ht2g_cur_bss; 
	switch (wid) {
	case PRI_RSSSI:
		parent_weight =
		&cur_bss_info->cur_weight_info.pri_rssi_weight;
		break;
	case PRI_BSS_CNT:
		parent_weight =
		&cur_bss_info->cur_weight_info.pri_count_weight;
		break;
	case PRI_NOISE_FLOOR:
		parent_weight =
		&cur_bss_info->cur_weight_info.pri_noise_floor_weight;
		break;
	case SEC_RSSSI:
		parent_weight =
		&cur_bss_info->cur_weight_info.sec_rssi_weight;
		break;
	case SEC_BSS_CNT:
		parent_weight =
		&cur_bss_info->cur_weight_info.sec_count_weight;
		break;
	case SEC_NOISE_FLOOR:
		parent_weight =
		&cur_bss_info->cur_weight_info.sec_noise_floor_weight;
		break;
	default:
		avg_weight = weight;
		return avg_weight;
	}
	avg_weight = moving_avg(weight, *parent_weight, 14);
	*parent_weight = avg_weight;

	return avg_weight;
}

static int write_wcn_key_log(const char *log, wcn_key_log_id log_id)
{
	struct kernel_packet_info *user_msg_info;
	char log_tag_scn[32] = "wifi_fool_proof";
	unsigned char *log_buf = NULL;
	unsigned int kevent_len = 0, log_len = 0;

	log_len = strlen(log) + 1;
	kevent_len = log_len + sizeof(struct kernel_packet_info);
	log_buf = qdf_mem_malloc_atomic(kevent_len);
	if (!log_buf) {
		smbw_err("smbw--mem_malloc failed!! %s, %d",
			__func__, __LINE__);
		return -1;
	}
	user_msg_info = (struct kernel_packet_info *)log_buf;
	user_msg_info->type = 1;
	snprintf(user_msg_info->log_tag, sizeof(user_msg_info->log_tag),
		"%s", log_tag_scn);
	switch(log_id) {
	case scan_band_ratio:
		snprintf(user_msg_info->event_id,
			sizeof(user_msg_info->event_id),
			"%s", logid2str(log_id));
		break;
	case connect_band_ratio:
		snprintf(user_msg_info->event_id,
			sizeof(user_msg_info->event_id),
			"%s", logid2str(log_id));
		break;
	default:
		snprintf(user_msg_info->event_id,
			sizeof(user_msg_info->event_id),
			"%s", logid2str(connect_band_ratio));
		break;
        }
	snprintf(user_msg_info->payload, log_len, "%s", log);
	user_msg_info->payload_length = log_len;
	kevent_send_to_user(user_msg_info);
	qdf_mem_free(log_buf);

	return 0;
}

static int post_kevent_log(struct smart_bw_mgr *bw_mgr)
{
	unsigned char log_buf[320];
	unsigned char ht40_chan_busy = 0, bg_bss_num = 0;
	unsigned char ht40_intolerant_num = 0, exist_ht40_conflict = 0;
	unsigned char cur_chan_free_weight, chan_free_weight;
	unsigned char rssi_weight, count_weight, noise_floor_weight;
	struct ht2g_bss_info *cur_bss_info = NULL;

	cur_bss_info = &bw_mgr->ht2g_cur_bss;
	if (!bw_mgr->ht40_connected) {
		smbw_debug("smbw--no ht40_connected!! %s, %d",
		__func__, __LINE__);
		return -1;
	}
	if (!cur_bss_info->need_post_lq_log) {
		smbw_debug("smbw--no need_post_lq_log!! %s, %d",
		__func__, __LINE__);
		return -1;
	}

	rssi_weight =
	cur_bss_info->cur_weight_info.pri_rssi_weight +
	cur_bss_info->cur_weight_info.sec_rssi_weight;
	count_weight =
	cur_bss_info->cur_weight_info.pri_count_weight +
	cur_bss_info->cur_weight_info.sec_count_weight;
	noise_floor_weight =
	cur_bss_info->cur_weight_info.pri_noise_floor_weight +
	cur_bss_info->cur_weight_info.sec_noise_floor_weight;
	cur_chan_free_weight = get_chan_free_weight(bw_mgr);
	chan_free_weight =
	moving_avg(cur_chan_free_weight,
		cur_bss_info->cur_weight_info.channel_free_weight,
		14);
	cur_bss_info->cur_weight_info.channel_free_weight =
		chan_free_weight;
	snprintf(log_buf, sizeof(log_buf),
	"[SMART_BW]:rx_nss:%d,avg_rx_mcs:%d,avg_w:%d,avg_rssi:%d,rssi_w:%d,cnt_w:%d,nf_w:%d,chanf_w:%d",
	bw_mgr->ll_stats_metric.rx_nss,
	bw_mgr->ll_stats_metric.avg_rx_mcs,
	bw_mgr->ll_stats_metric.adaptive_avg_weight,
	bw_mgr->ll_stats_metric.avg_rssi,
	rssi_weight, count_weight,
	noise_floor_weight, chan_free_weight);
	smbw_err("[SMART_BW]:rx_nss:%d,avg_rx_mcs:%d,avg_w:%d,avg_rssi:%d,rssi_w:%d,cnt_w:%d,nf_w:%d,chanf_w:%d,@%s,%d",
	bw_mgr->ll_stats_metric.rx_nss,
	bw_mgr->ll_stats_metric.avg_rx_mcs,
	bw_mgr->ll_stats_metric.adaptive_avg_weight,
	bw_mgr->ll_stats_metric.avg_rssi,
	rssi_weight, count_weight,
	noise_floor_weight, chan_free_weight, __func__, __LINE__);
	write_wcn_key_log(log_buf, connect_band_ratio);

	handle_exist_ht40bss(bw_mgr,
		cur_bss_info->chan_idx,
		cur_bss_info->sec_chan_idx);
	exist_ht40_conflict = bw_mgr->bw_decision.exist_ht40_conflict;
	ht40_chan_busy = check_ht40_chan_busy(bw_mgr);
	ht40_intolerant_num = get_total_num_intol(bw_mgr,
		cur_bss_info->af_chan_start,
		cur_bss_info->af_chan_end);
	bg_bss_num = get_total_bg_cnt(bw_mgr,
		cur_bss_info->af_chan_start,
		cur_bss_info->af_chan_end);
	snprintf(log_buf, sizeof(log_buf),
	"[SMART_BW]:chan_conflict:%d,chan_busy:%d,intolerant_num:%d,bg_bss_num:%d",
	exist_ht40_conflict, ht40_chan_busy, ht40_intolerant_num, bg_bss_num);

#if 1
	smbw_err("[SMART_BW]:chan_conflict:%d,chan_busy:%d,intolerant_num:%d,bg_bss_num:%d,@%s,%d",
	exist_ht40_conflict, ht40_chan_busy, ht40_intolerant_num, bg_bss_num,
	__func__, __LINE__);
#endif

	write_wcn_key_log(log_buf, scan_band_ratio);
	cur_bss_info->need_post_lq_log = 0;

	return 0;
}

static bool chan_sel_init(struct smart_bw_mgr *bw_mgr)
{
	tAcsChSelSpectInfo *pSpectInfoParams;
	tAcsSpectChInfo *pSpectCh = NULL;
	unsigned short channelnum = 0;

	smbw_debug("smbw--enter %s, %d", __func__, __LINE__);
	pSpectInfoParams = &bw_mgr->SpectInfoParams;
	pSpectInfoParams->numSpectChans = 14;
	
	pSpectCh = &pSpectInfoParams->pSpectCh[0];


        /* Fill the channel number in the spectrum in the operating freq band */
	for (channelnum = 0;
	channelnum < pSpectInfoParams->numSpectChans;
	channelnum++, pSpectCh++) {
		pSpectCh->chNum = channelnum + 1;
		pSpectCh->rssiAgr = ACS_MIN_RSSI;
		pSpectCh->channelWidth = 0;
		/* Initialise max ACS weight for all channels */
		pSpectCh->weight = ACS_WEIGHT_MAX;
		pSpectCh->valid = true;
	}
	smbw_debug("smbw--channelnum:%d, @%s, %d", channelnum, __func__, __LINE__);
	return true;
}

static void clear_SpectInfoParams(struct smart_bw_mgr *bw_mgr)
{
	tAcsSpectChInfo *pSpectCh = NULL;
	tAcsChSelSpectInfo *pSpectInfoParams;
	int i;

	pSpectInfoParams = &bw_mgr->SpectInfoParams;
	pSpectCh = &pSpectInfoParams->pSpectCh[0];
	for (i = 0; i < pSpectInfoParams->numSpectChans; i++) {
		pSpectCh->channelWidth = 0;
		pSpectCh->bssCount = 0;
		pSpectCh->weight = ACS_WEIGHT_MAX;
		pSpectCh->rssiAgr = ACS_MIN_RSSI;
		pSpectCh->weight_copy = 0;
		pSpectCh++;
	}
}

static unsigned int weight_rssi_count(char rssi, tAcsSpectChInfo *pSpectCh)
{
	int rssiWeight = 0;
	int countWeight = 0;
	unsigned short count = 0;
	unsigned int rssicountWeight = 0;
	struct ht2g_bss_info *cur_bss_info = NULL;
	unsigned char acs_rssi_weight_cfg, acs_count_weight_cfg;
	unsigned char acs_rssi_weight_local, acs_count_weight_local;

	count = pSpectCh->bssCount;
	acs_rssi_weight_cfg =
		ACS_WEIGHT_RSSI_CFG(ACS_WEIGHT);

	acs_count_weight_cfg =
		ACS_WEIGHT_COUNT_CFG(ACS_WEIGHT);

	acs_rssi_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT, acs_rssi_weight_cfg);

	acs_count_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT, acs_count_weight_cfg);

	rssiWeight = ACS_WEIGHT_COMPUTE(ACS_WEIGHT,
		acs_rssi_weight_cfg,
		rssi - ACS_MIN_RSSI,
		ACS_MAX_RSSI - ACS_MIN_RSSI);

	if (rssiWeight > acs_rssi_weight_local)
		rssiWeight = acs_rssi_weight_local;
	else if (rssiWeight < 0)
		rssiWeight = 0;

	countWeight = ACS_WEIGHT_COMPUTE(ACS_WEIGHT, acs_count_weight_cfg,
		count - ACS_MIN_COUNT,
		ACS_MAX_COUNT - ACS_MIN_COUNT);

	if (countWeight > acs_count_weight_local)
		countWeight = acs_count_weight_local;

	rssicountWeight = rssiWeight + countWeight;

	if (smart_bw_mgr_g.ht40_connected) {
		cur_bss_info = &smart_bw_mgr_g.ht2g_cur_bss;
		if (pSpectCh->chNum == cur_bss_info->chan_idx) {
			cal_avg_weight(PRI_RSSSI, rssiWeight);
			cal_avg_weight(PRI_BSS_CNT, countWeight);
			smbw_debug("smbw--pri_rssi_w:%d,pri_count_w:%d,@%s,%d",
				cur_bss_info->cur_weight_info.pri_rssi_weight,
				cur_bss_info->cur_weight_info.pri_count_weight,
				__func__, __LINE__);

		} else if (pSpectCh->chNum == cur_bss_info->sec_chan_idx) {
			cal_avg_weight(SEC_RSSSI, rssiWeight);
			cal_avg_weight(SEC_BSS_CNT, countWeight);
			smbw_debug("smbw--sec_rssi_w:%d,sec_count_w:%d,@%s, %d",
				cur_bss_info->cur_weight_info.sec_rssi_weight,
				cur_bss_info->cur_weight_info.sec_count_weight,
				__func__, __LINE__);
		}
	}
	smbw_debug("smbw--rssicountWeight:%d,rssiWeight:%d,countWeight:%d,@%s, %d",
		rssicountWeight, rssiWeight, countWeight, __func__, __LINE__);
	return rssicountWeight;
}

static unsigned int
weight_channel_noise_floor(struct lim_channel_status *channel_stat)
{
	unsigned int noise_floor_weight;
	unsigned char acs_nf_weight_cfg;
	unsigned char acs_nf_weight_local, chan_id = 0;
	struct ht2g_bss_info *cur_bss_info = NULL;

	acs_nf_weight_cfg =
		ACS_WEIGHT_NOISE_FLOOR_CFG(ACS_WEIGHT);

	acs_nf_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT,acs_nf_weight_cfg);

	if (!channel_stat || channel_stat->channelfreq == 0) {
		smbw_debug("smbw--sanity check failed return max weight,@%s,%d",
			__func__, __LINE__);
		return acs_nf_weight_local;
	}

	noise_floor_weight = (channel_stat->noise_floor == 0) ? 0 :
		(ACS_WEIGHT_COMPUTE(ACS_WEIGHT, acs_nf_weight_cfg,
		channel_stat->noise_floor - ACS_MIN_NF,
		ACS_MAX_NF - ACS_MIN_NF));

	if (noise_floor_weight > acs_nf_weight_local)
		noise_floor_weight = acs_nf_weight_local;
        
	smbw_debug("smbw--nf=%d, nfwc=%d, nfwl=%d, nfw=%d,@%s,%d",
		channel_stat->noise_floor, acs_nf_weight_cfg,
		acs_nf_weight_local, noise_floor_weight,
		__func__, __LINE__);

	if (smart_bw_mgr_g.ht40_connected) {
		cur_bss_info = &smart_bw_mgr_g.ht2g_cur_bss;
		chan_id = freq_to_chan_2g(channel_stat->channelfreq);
		if (chan_id == cur_bss_info->chan_idx) {
			cal_avg_weight(PRI_NOISE_FLOOR, noise_floor_weight);
			smbw_debug("smbw--pri_nf_w:%d,@%s, %d",
				cur_bss_info->cur_weight_info.pri_noise_floor_weight,
				__func__, __LINE__);

		} else if (chan_id == cur_bss_info->sec_chan_idx) {
			cal_avg_weight(SEC_NOISE_FLOOR, noise_floor_weight);
			smbw_debug("smbw--sec_nf_w:%d, @%s, %d",
				cur_bss_info->cur_weight_info.sec_noise_floor_weight,
				__func__, __LINE__);
		}
	}
	smbw_debug("smbw--noise_floor_weight:%d, @%s, %d",
		noise_floor_weight,
		__func__, __LINE__);
	return noise_floor_weight;
}

static unsigned int
weight_channel_free(struct lim_channel_status *channel_stat)
{
	unsigned int channel_free_weight, rx_clear_count = 0;
	unsigned int cycle_count = 0;
	unsigned char acs_channel_free_weight_cfg;
	unsigned char acs_channel_free_weight_local;
	unsigned char chan_id;

	acs_channel_free_weight_cfg =
		ACS_WEIGHT_CHANNEL_FREE_CFG(ACS_WEIGHT);

	acs_channel_free_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT,acs_channel_free_weight_cfg);

	if (!channel_stat || channel_stat->channelfreq == 0) {
		smbw_err("smbw--check failed return max weight,@%s, %d",
			__func__, __LINE__);
		return acs_channel_free_weight_local;
	}

	rx_clear_count = channel_stat->rx_clear_count -
		channel_stat->tx_frame_count -
		channel_stat->rx_frame_count;
	cycle_count = channel_stat->cycle_count;

	channel_free_weight = (cycle_count == 0) ? 0 :
		(ACS_WEIGHT_COMPUTE(
		ACS_WEIGHT,
		acs_channel_free_weight_cfg,
		((rx_clear_count << 8) +
		(cycle_count >> 1))/cycle_count -
		(ACS_MIN_CHNFREE << 8),
		(ACS_MAX_CHNFREE -
		ACS_MIN_CHNFREE) << 8));

	if (channel_free_weight > acs_channel_free_weight_local)
		channel_free_weight = acs_channel_free_weight_local;

	chan_id = freq_to_chan_2g(channel_stat->channelfreq);
	smart_bw_mgr_g.chan_free_weight[chan_id - 1] = channel_free_weight;

        smbw_debug("smbw--rcc=%d, cc=%d, tc=%d, rc=%d, cfwc=%d, cfwl=%d, cfw=%d,@%s, %d",
                rx_clear_count, cycle_count,
                channel_stat->tx_frame_count,
                channel_stat->rx_frame_count,
                acs_channel_free_weight_cfg,
                acs_channel_free_weight_local,
                channel_free_weight,
                __func__, __LINE__);

        return channel_free_weight;
}

static unsigned int
weight_channel_txpwr_range(struct lim_channel_status *channel_stat)
{
	unsigned int txpwr_weight_low_speed;
	unsigned char acs_txpwr_range_weight_cfg;
	unsigned char acs_txpwr_range_weight_local;

	acs_txpwr_range_weight_cfg =
		ACS_WEIGHT_TX_POWER_RANGE_CFG(ACS_WEIGHT);

	acs_txpwr_range_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT,acs_txpwr_range_weight_cfg);

	if (!channel_stat || channel_stat->channelfreq == 0) {
		smbw_debug("smbw--check failed return max weight,@%s,%d",
			__func__, __LINE__);
		return acs_txpwr_range_weight_local;
	}

	txpwr_weight_low_speed = (channel_stat->chan_tx_pwr_range == 0) ? 0 :
		(ACS_WEIGHT_COMPUTE(
		ACS_WEIGHT,
		acs_txpwr_range_weight_cfg,
		ACS_MAX_TXPWR -
		channel_stat->chan_tx_pwr_range,
		ACS_MAX_TXPWR - ACS_MIN_TXPWR));

	if (txpwr_weight_low_speed > acs_txpwr_range_weight_local)
		txpwr_weight_low_speed = acs_txpwr_range_weight_local;

	smbw_debug("smbw--tpr=%d, tprwc=%d, tprwl=%d, tprw=%d,@%s,%d",
		channel_stat->chan_tx_pwr_range,
		acs_txpwr_range_weight_cfg,
		acs_txpwr_range_weight_local,
		txpwr_weight_low_speed,
		__func__, __LINE__);

	return txpwr_weight_low_speed;
}

static unsigned int
weight_channel_txpwr_tput(struct lim_channel_status *channel_stat)
{
	unsigned int txpwr_weight_high_speed;
	unsigned char acs_txpwr_tput_weight_cfg;
	unsigned char acs_txpwr_tput_weight_local;

	acs_txpwr_tput_weight_cfg =
		ACS_WEIGHT_TX_POWER_THROUGHPUT_CFG(ACS_WEIGHT);

	acs_txpwr_tput_weight_local =
		ACS_WEIGHT_CFG_TO_LOCAL(ACS_WEIGHT, acs_txpwr_tput_weight_cfg);

	if (!channel_stat || channel_stat->channelfreq == 0) {
		smbw_debug("smbw--check failed return max weight,@%s,%d",
			__func__, __LINE__);
		return acs_txpwr_tput_weight_local;
	}

	txpwr_weight_high_speed = (channel_stat->chan_tx_pwr_throughput == 0) ? 0 :
		(ACS_WEIGHT_COMPUTE(
		ACS_WEIGHT,
		acs_txpwr_tput_weight_cfg,
		ACS_MAX_TXPWR -
		channel_stat->chan_tx_pwr_throughput,
		ACS_MAX_TXPWR - ACS_MIN_TXPWR));

	if (txpwr_weight_high_speed > acs_txpwr_tput_weight_local)
		txpwr_weight_high_speed = acs_txpwr_tput_weight_local;

	smbw_debug("smbw--tpt=%d, tptwc=%d, tptwl=%d, tptw=%d,@%s,%d",
		channel_stat->chan_tx_pwr_throughput,
		acs_txpwr_tput_weight_cfg,
		acs_txpwr_tput_weight_local,
		txpwr_weight_high_speed,
		__func__, __LINE__);

	return txpwr_weight_high_speed;
}

static unsigned int
weight_channel_status(struct lim_channel_status *channel_stat)
{
#if 0
	smbw_err("smbw--channelfreq:%d, cycle_count:%d, @%s,%d",
		channel_stat->channelfreq, channel_stat->cycle_count, __func__, __LINE__);
#endif
	return weight_channel_noise_floor(channel_stat) +
		weight_channel_free(channel_stat) +
		weight_channel_txpwr_range(channel_stat) +
		weight_channel_txpwr_tput(channel_stat);
}

static void update_rssi_bsscount(tAcsSpectChInfo *pSpectCh, int offset,
	tAcsSpectChInfo *spectch_start,
	tAcsSpectChInfo *spectch_end)
{
	tAcsSpectChInfo *pExtSpectCh = NULL;
	int rssi, rsssi_effect;

	pExtSpectCh = (pSpectCh + offset);
	if (pExtSpectCh &&
		pExtSpectCh >= spectch_start &&
		pExtSpectCh < spectch_end) {
		++pExtSpectCh->bssCount;
		switch (offset) {
		case -1:
		case 1:
			rsssi_effect =
			ACS_24GHZ_FIRST_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY;
			break;
		case -2:
		case 2:
			rsssi_effect =
			ACS_24GHZ_SEC_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY;
			break;
		case -3:
		case 3:
			rsssi_effect =
			ACS_24GHZ_THIRD_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY;
			break;
		case -4:
		case 4:
			rsssi_effect =
			ACS_24GHZ_FOURTH_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY;
			break;
		default:
			rsssi_effect = 0;
			break;
		}

		rssi = pSpectCh->rssiAgr + rsssi_effect;
		if (IS_RSSI_VALID(pExtSpectCh->rssiAgr, rssi))
			pExtSpectCh->rssiAgr = rssi;
		if (pExtSpectCh->rssiAgr < ACS_MIN_RSSI)
			pExtSpectCh->rssiAgr = ACS_MIN_RSSI;
#if 0
		smbw_debug("smbw--rssiAgr:%d @%s, %d",
			pExtSpectCh->rssiAgr, __func__, __LINE__);
#endif
	}
}

static void clear_chan_stats(struct smart_bw_mgr *bw_mgr)
{
	qdf_spin_lock(&bw_mgr->chan_stats_lock);
	qdf_mem_set(&bw_mgr->chan_stats[0],
	sizeof(struct lim_channel_status) * 14, 0);
	qdf_spin_unlock(&bw_mgr->chan_stats_lock);
	bw_mgr->handle_chan_stats_going = 0;
	smbw_debug("smbw--enter %s, %d", __func__, __LINE__);
}

static void
interference_rssi_count(tAcsSpectChInfo *spect_ch,
	tAcsSpectChInfo *spectch_start,
	tAcsSpectChInfo *spectch_end)
{
	if (!spect_ch) {
		smbw_err("smbw--%s: spect_ch is NULL", __func__);
		return;
	}

	switch (spect_ch->chNum) {
	case CHANNEL_1:
		update_rssi_bsscount(spect_ch, 1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 4,spectch_start, spectch_end);
		break;
	case CHANNEL_2:
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 4,spectch_start, spectch_end);
		break;
	case CHANNEL_3:
		update_rssi_bsscount(spect_ch, -2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 4, spectch_start, spectch_end);
		break;
	case CHANNEL_4:
		update_rssi_bsscount(spect_ch, -3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 4, spectch_start, spectch_end);
		break;
	case CHANNEL_5:
	case CHANNEL_6:
	case CHANNEL_7:
	case CHANNEL_8:
	case CHANNEL_9:
	case CHANNEL_10:
		update_rssi_bsscount(spect_ch, -4, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -3, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 4, spectch_start, spectch_end);
		break;
	case CHANNEL_11:
		update_rssi_bsscount(spect_ch, -4,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 3, spectch_start, spectch_end);
		break;
	case CHANNEL_12:
		update_rssi_bsscount(spect_ch, -4,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 2,spectch_start, spectch_end);
		break;
	case CHANNEL_13:
		update_rssi_bsscount(spect_ch, -4,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -3,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, 1,spectch_start, spectch_end);
		break;
	case CHANNEL_14:
		update_rssi_bsscount(spect_ch, -4,spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -3, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -2, spectch_start, spectch_end);
		update_rssi_bsscount(spect_ch, -1, spectch_start, spectch_end);
		break;
	default:
		break;
	}
}

static void cal_spect_weight(tAcsChSelSpectInfo *pSpectInfoParams,
	struct smart_bw_mgr *bw_mgr,
	qdf_list_t *scan_list)
{
	char rssi = 0;
	unsigned char chn_num = 0, chan_id = 0;
	tAcsSpectChInfo *pSpectCh = NULL;
	tAcsSpectChInfo *spectch_start = NULL;
	tAcsSpectChInfo *spectch_end = NULL;
	qdf_list_node_t *cur_lst = NULL, *next_lst = NULL;
	struct scan_cache_node *cur_node = NULL;

	smbw_debug("smbw--enter+1 %s, %d", __func__, __LINE__);
	pSpectCh = &pSpectInfoParams->pSpectCh[0];
	spectch_start = &pSpectInfoParams->pSpectCh[0];
	spectch_end = &pSpectInfoParams->pSpectCh[bw_mgr->chan_stats_cnt];
	if (!scan_list) {
		smbw_err("smbw--NULL pointer! @%s, %d", __func__, __LINE__);
		return;
	}

	if (scan_list)
		qdf_list_peek_front(scan_list, &cur_lst);
	while (cur_lst) {
		cur_node = qdf_container_of(cur_lst, struct scan_cache_node, node);
		chan_id = freq_to_chan_2g(cur_node->entry->channel.chan_freq);
		if (chan_id < 1 || chan_id > 14 ||
			cur_node->entry->rssi_raw < ACS_MIN_RSSI) {
			qdf_list_peek_next(scan_list, cur_lst, &next_lst);
			cur_lst = next_lst;
			next_lst = NULL;
			continue;
		}
		pSpectCh = &pSpectInfoParams->pSpectCh[0];
		for (chn_num = 0;
			chn_num < bw_mgr->chan_stats_cnt;
			chn_num++) {
			if (pSpectCh && (chan_id == pSpectCh->chNum)) {
				if (pSpectCh->rssiAgr <
					cur_node->entry->rssi_raw)
					pSpectCh->rssiAgr =
					cur_node->entry->rssi_raw;
					++pSpectCh->bssCount;
					interference_rssi_count(pSpectCh,
						spectch_start, 
						spectch_end);
					pSpectCh++;
				break;
			} else {
				pSpectCh++;
			}
		}
		qdf_list_peek_next(scan_list, cur_lst, &next_lst);
		cur_lst = next_lst;
		next_lst = NULL;
	}
	/* Calculate the weights for all channels in the spectrum pSpectCh */
	pSpectCh = &pSpectInfoParams->pSpectCh[0];
	for (chn_num = 0; chn_num < bw_mgr->chan_stats_cnt; chn_num++) {
		/*
		   rssi : Maximum received signal strength among all BSS on that channel
		   bssCount : Number of BSS on that channel
		 */
		rssi = (char) pSpectCh->rssiAgr;
		if (rssi < ACS_MIN_RSSI)
			rssi = ACS_MIN_RSSI;
		if (pSpectCh->weight > ACS_WEIGHT_MAX) {
			pSpectCh->weight = ACS_WEIGHT_MAX;
			pSpectCh->weight_copy = pSpectCh->weight;
			goto debug_info;
		}

		pSpectCh->weight =
		ACS_NORMALISE_1000 *
		(weight_rssi_count(rssi, pSpectCh) +
		weight_channel_status(&bw_mgr->chan_stats[pSpectCh->chNum - 1]));

		if (pSpectCh->weight > ACS_WEIGHT_MAX)
			pSpectCh->weight = ACS_WEIGHT_MAX;
		pSpectCh->weight_copy = pSpectCh->weight;

debug_info:
		smbw_debug("smbw--Chan=%d,Weight= %d,rssiAgr=%d,bssCount=%d,@%s,%d",
			pSpectCh->chNum,pSpectCh->weight,pSpectCh->rssiAgr,
			pSpectCh->bssCount,__func__, __LINE__);
		pSpectCh++;
	}
}

static void cal_chl_weight_ht40_24_g(struct smart_bw_mgr *bw_mgr)
{
	unsigned char pri_chan = 0, sec_chan = 0;

	pri_chan = bw_mgr->ht2g_cur_bss.chan_idx;
	sec_chan = bw_mgr->ht2g_cur_bss.sec_chan_idx;
	qdf_spin_lock(&bw_mgr->chan_weight_lock);
	bw_mgr->cur_weight =
	bw_mgr->chan_weight[pri_chan - 1] + bw_mgr->chan_weight[sec_chan - 1];
	bw_mgr->cur_weight_timestamp = bw_mgr->chan_weight_timestamp;
	qdf_spin_unlock(&bw_mgr->chan_weight_lock);

	smbw_debug("smbw--pri_chan:%d,sec_chan:%d,cur_weight:%d, @%s, %d",
		pri_chan, sec_chan, bw_mgr->cur_weight,
		__func__, __LINE__);
}

static unsigned int get_cur_chan_stats_num(struct smart_bw_mgr *bw_mgr)
{
	struct lim_channel_status *chan_stat = NULL;
	unsigned int chan_stats_num = 0, i;

	chan_stat = &bw_mgr->chan_stats[0];
	for (i = 0; i < 14; i++) {
		if (chan_stat->channelfreq > 0)
			chan_stats_num += 1;
		chan_stat++;
	}
	bw_mgr->chan_stats_cnt = chan_stats_num;
	return chan_stats_num;
}

static void chan_stats_wake_up(struct smart_bw_mgr *bw_mgr)
{
	bw_mgr->chan_stats_wake = 1;
	bw_mgr->last_chan_stat_freq = 0;
	bw_mgr->handle_chan_stats_going = 1;
	wake_up(&bw_mgr->chan_stats_waitq);
}

static int get_config_rom_update(void);
int smart_bw_deinit(void);

int handle_scan_chan_stats(struct wlan_objmgr_pdev *pdev,
	wmi_chan_info_event_fixed_param *event)
{
	unsigned char chan_id = 0;
	struct lim_channel_status *chan_stat = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit)) {
		smbw_debug("smbw--feature disabled or func_exit! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (!pdev || !event) {
		smbw_err("smbw--NULL pointer @%s, %d", __func__, __LINE__);
		return -1;
	}
	bw_mgr = &smart_bw_mgr_g;

	if (bw_mgr->handle_chan_stats_going == 1) {
		smbw_err("smbw--handle_chan_stats_going @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	chan_stat = &bw_mgr->chan_stats[0];
	if (event->freq != 0) {
		if (event->freq < WLAN_24_GHZ_BASE_FREQ ||
			event->freq > WLAN_CHAN_14_FREQ)
			return 0;
		bw_mgr->chan_stats_wake = 0;
		bw_mgr->pdev = pdev;
		bw_mgr->last_chan_stat_freq = event->freq;
		chan_id = freq_to_chan_2g(event->freq);
		qdf_spin_lock(&bw_mgr->chan_stats_lock);
		chan_stat[chan_id - 1].channelfreq = event->freq;
		chan_stat[chan_id - 1].noise_floor = event->noise_floor;
		chan_stat[chan_id - 1].rx_clear_count = event->rx_clear_count;
		chan_stat[chan_id - 1].cycle_count = event->cycle_count;
		chan_stat[chan_id - 1].chan_tx_pwr_range =
			event->chan_tx_pwr_range;
		chan_stat[chan_id - 1].chan_tx_pwr_throughput =
			event->chan_tx_pwr_tp;
		chan_stat[chan_id - 1].rx_frame_count = event->rx_frame_count;
		chan_stat[chan_id - 1].bss_rx_cycle_count =
			event->my_bss_rx_cycle_count;
		chan_stat[chan_id - 1].rx_11b_mode_data_duration =
			event->rx_11b_mode_data_duration;
		chan_stat[chan_id - 1].tx_frame_count = event->tx_frame_cnt;
		chan_stat[chan_id - 1].mac_clk_mhz = event->mac_clk_mhz;
		chan_stat[chan_id - 1].channel_id = chan_id;
		chan_stat[chan_id - 1].cmd_flags = event->cmd_flags;
		qdf_spin_unlock(&bw_mgr->chan_stats_lock);
	} else {
		if (get_cur_chan_stats_num(bw_mgr) >= 11 &&
			!bw_mgr->handle_chan_stats_going) {
			chan_stats_wake_up(bw_mgr);
			queue_work(bw_mgr->chan_stats_queue,
				&bw_mgr->chan_stats_work);
		}
	}
	return 0;
}

static int check_ht40_intolerant(unsigned short ht_cap)
{
	if (ht_cap & HT_CAP_INFO_40MHZ_INTOLERANT)
		return 1;
	else
		return 0;
}

static void handle_legacy_bss(struct smart_bw_mgr *bw_mgr,
	unsigned char chan_start,
	unsigned char chan_end)
{
	unsigned int total_bg_cnt = 0;

	total_bg_cnt = get_total_bg_cnt(bw_mgr, chan_start, chan_end);

	if (total_bg_cnt > BG_SCORE_THRES) {
		bw_mgr->bw_decision.ht40_bg_enable = 0;
	} else {
		bw_mgr->bw_decision.ht40_bg_enable = 1;
	}
	g_last_connect_record.ht40_bg_enable =
		bw_mgr->bw_decision.ht40_bg_enable;
	smbw_debug("smbw--total_bg_score:%d,ht40_bg_enable:%d, @%s, %d",
		total_bg_cnt, bw_mgr->bw_decision.ht40_bg_enable,
		__func__, __LINE__);
}

static unsigned char handle_intol_res(struct smart_bw_mgr *bw_mgr,
	unsigned char chan_start,
	unsigned char chan_end)
{
	unsigned char total_num_intol = 0;

	total_num_intol = get_total_num_intol(bw_mgr, chan_start, chan_end);
	if (total_num_intol > INTOLERANT_NUM_THRES) {
		bw_mgr->bw_decision.ht40_intolerant = 1;
	} else {
		bw_mgr->bw_decision.ht40_intolerant = 0;
	}
	g_last_connect_record.ht40_intolerant =
		bw_mgr->bw_decision.ht40_intolerant;
	if (total_num_intol > 0) {
		smbw_debug("smbw--total_num_intol:%d, @%s, %d",
			total_num_intol, __func__, __LINE__);
	}
	return total_num_intol;
}

static void process_scan_res(struct smart_bw_mgr *bw_mgr)
{
	struct ht2g_bss_info *cur_bss_info = NULL;

	cur_bss_info = &bw_mgr->ht2g_cur_bss;
	handle_legacy_bss(bw_mgr,
		cur_bss_info->af_chan_start,
		cur_bss_info->af_chan_end);
	handle_intol_res(bw_mgr,
		cur_bss_info->af_chan_start,
		cur_bss_info->af_chan_end);
	handle_exist_ht40bss(bw_mgr,
		cur_bss_info->chan_idx,
		cur_bss_info->sec_chan_idx);
}

static void make_scan_bw_decision(struct smart_bw_mgr *bw_mgr)
{
	if (!bw_mgr->bw_decision.exist_ht40_conflict &&
		!bw_mgr->bw_decision.ht40_intolerant &&
		bw_mgr->bw_decision.ht40_bg_enable) {
		bw_mgr->bw_decision.ht40_scan_enable = 1;
	} else {
		bw_mgr->bw_decision.ht40_scan_enable = 0;
	}

	smbw_debug("smbw--scan_enable:%d,intolerant:%d,bg_enable:%d,@%s, %d",
	bw_mgr->bw_decision.ht40_scan_enable,
	bw_mgr->bw_decision.ht40_intolerant,
	bw_mgr->bw_decision.ht40_bg_enable,
	__func__, __LINE__);
}

static int clear_res_stats(struct smart_bw_mgr *bw_mgr)
{
	memset(bw_mgr->res_stats, 0, sizeof(struct scan_res_stats));
	return 0;
}

static void
copy_ht40_bss(struct scan_cache_node *cur_node,
	struct scan_res_stats *res_stats, int chan_idx, int chan_offset)
{
	unsigned char num = 0;
	ht40_bss_t *ht40_bss = NULL;
	int sec_chan = 0;

	if (smart_bw_mgr_g.scan_results.ht40_bss_num >= 20)
		return;

	if (chan_offset == 1) {
		sec_chan = chan_idx + 4;
		if (sec_chan < 1 || sec_chan > 14) {
			smbw_err("smbw--invalid sec chan offset! @%s, %d",
			__func__, __LINE__);
			return;
		}
		res_stats->ht40_chan_bssinfo[chan_idx + 4].nExt20 += 1;
	} else if (chan_offset == -1) {
		sec_chan = chan_idx - 4;
		if (sec_chan < 1 || sec_chan > 14) {
			smbw_err("smbw--invalid sec chan offset! @%s, %d",
			__func__, __LINE__);
			return;
		}
		res_stats->ht40_chan_bssinfo[chan_idx - 4].nExt20 += 1;
	} else {
		smbw_err("smbw--invalid chan_offset! @%s, %d",
		__func__, __LINE__);
		return;
	}
	res_stats->ht40_chan_bssinfo[chan_idx].nCtrl += 1;
	res_stats->ht40_bss_num += 1;

	num = smart_bw_mgr_g.scan_results.ht40_bss_num;
	ht40_bss = &smart_bw_mgr_g.scan_results.ht40_bss[num];
	memcpy(&ht40_bss->bssid[0], &cur_node->entry->bssid.bytes[0],
		ETH_ALEN);
	ht40_bss->rssi_raw = cur_node->entry->rssi_raw;
	ht40_bss->chan_offset = chan_offset;
	ht40_bss->chan_id =
		freq_to_chan_2g(cur_node->entry->channel.chan_freq);
	smart_bw_mgr_g.scan_results.ht40_bss_num += 1;
	smbw_err("smbw--HT40PLUS, pri_chan:%d, sec_chan:%d, rssi_raw:%d, @%s, %d",
		chan_idx, sec_chan, ht40_bss->rssi_raw,
		__func__, __LINE__);

}

static int handle_scan_res(qdf_list_t *scan_res_list)
{
	struct scan_cache_node *cur_node = NULL;
	struct scan_cache_node *next_node = NULL;
	unsigned int bss_num = 0;
	unsigned char chan_idx = 0;
	unsigned short ht_cap = 0;
	struct ht_common_ie *htcap = NULL;
	struct scan_res_stats *res_stats = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;

	bw_mgr = &smart_bw_mgr_g;
	res_stats = bw_mgr->res_stats;

	if (!scan_res_list) {
		smbw_err("smbw--get scan result failed, @%s, %d",
		__func__, __LINE__);
		return -1;
	} else {
		smbw_debug("smbw--num of scan results is %d, @%s, %d",
			qdf_list_size(scan_res_list), __func__, __LINE__);
	}
	qdf_list_peek_front(scan_res_list,(qdf_list_node_t **) &cur_node);
	if (!cur_node) {
		smbw_err("smbw--cur_node is NULL @%s, %d", __func__, __LINE__);
		return -1;
	}

	qdf_spin_lock(&bw_mgr->scan_stats_lock);
	bw_mgr->scan_results.ht40_bss_num = 0;
	clear_res_stats(bw_mgr);
	while (cur_node) {
		chan_idx = freq_to_chan_2g(cur_node->entry->channel.chan_freq);
		if (chan_idx >= 1 && chan_idx <= MAX_2G_CHAN_NUM &&
			cur_node->entry->rssi_raw > LOW_RSSI_THRES) {
			smbw_debug("smbw--cur_node->phy_mode:%d, rssi_raw:%d, ssid:%s, @%s, %d",
				cur_node->entry->phy_mode,
				cur_node->entry->rssi_raw,
				cur_node->entry->ssid.ssid,
				__func__, __LINE__);
			switch(cur_node->entry->phy_mode) {
			case WLAN_PHYMODE_11B:
			case WLAN_PHYMODE_11G:
			case WLAN_PHYMODE_11G_ONLY:
				res_stats->bg_bss_num[chan_idx] += 1;
				break;
			case WLAN_PHYMODE_11NG_HT20:
				if (cur_node->entry->ie_list.htcap) {
					htcap = (struct ht_common_ie *)(cur_node->entry->ie_list.htcap);
					if (htcap)
						ht_cap = le16toh(htcap->hc_cap);
					if (check_ht40_intolerant(ht_cap))
						bw_mgr->res_stats->ht40_intolerant[chan_idx] += 1;
				}
				break;
			default:
				if (IS_WLAN_PHYMODE_40MHZ(cur_node->entry->phy_mode)) {
					if (cur_node->entry->channel.cfreq0 >
						cur_node->entry->channel.chan_freq)
						copy_ht40_bss(cur_node, res_stats, chan_idx, 1);
					else
						copy_ht40_bss(cur_node, res_stats, chan_idx, -1);
				}
				break;
			}
		}
		qdf_list_peek_next(scan_res_list,
			(qdf_list_node_t *) cur_node,
			(qdf_list_node_t **) &next_node);
		cur_node = next_node;
		next_node = NULL;
		bss_num++;
	}
	qdf_spin_unlock(&bw_mgr->scan_stats_lock);
	bw_mgr->scan_results.bss_num = bss_num;
	return 0;
}

static void ll_stats_wake_up(struct smart_bw_mgr *bw_mgr)
{
	bw_mgr->ll_stats_wake = 1;
	smbw_debug("smbw--ll_stats_wake:%d, enter @%s, %d",
		bw_mgr->ll_stats_wake, __func__, __LINE__);
	wake_up(&bw_mgr->ll_stats_waitq);
}

static void make_chan_bw_decision(struct smart_bw_mgr *bw_mgr)
{
	unsigned int avg_cur_weight = 0, ht40_chan_busy = 0;
	ktime_t cur_time;

	if (bw_mgr->cur_weight == 0) {
		smbw_err("smbw--cur_weight is 0!! @%s, %d",
			__func__, __LINE__);
		bw_mgr->bw_decision.ht40_enable = 0;
		return;
	}

	cur_time = ktime_get();
	if ((ktime_to_ms(cur_time) -
	ktime_to_ms(bw_mgr->cur_weight_timestamp)) > AGE_30S) {
		bw_mgr->cur_weight = FULL_WEIGHT;
		smbw_err("smbw--cur_weight aged!! @%s, %d",
			__func__, __LINE__);
		bw_mgr->bw_decision.ht40_enable = 0;
		return;

	}
	if ((ktime_to_ms(cur_time) -
	ktime_to_ms(bw_mgr->parent_weight_timestamp)) > (AGE_30S + 15)) {
		bw_mgr->parent_cur_weight = 0;
	}

	avg_cur_weight = moving_avg(bw_mgr->cur_weight,
		bw_mgr->parent_cur_weight, g_acs_threshold_cfg.mov_avg_beta);
	bw_mgr->parent_cur_weight = avg_cur_weight;
	bw_mgr->parent_weight_timestamp = bw_mgr->cur_weight_timestamp;
	ht40_chan_busy = check_ht40_chan_busy(bw_mgr);

	if (avg_cur_weight < g_acs_threshold_cfg.acs_weight_thre &&
		bw_mgr->bw_decision.ht40_scan_enable &&
		!ht40_chan_busy)
		bw_mgr->bw_decision.ht40_chan_enable = 1;
	else
		bw_mgr->bw_decision.ht40_chan_enable = 0;

	if (bw_mgr->bw_decision.ht40_chan_enable)
		bw_mgr->bw_decision.ht40_enable = 1;
	else
		bw_mgr->bw_decision.ht40_enable = 0;

	smbw_err("smbw--ht40_enable:%d,chan_busy:%d,scan_enable:%d, @%s, %d",
		bw_mgr->bw_decision.ht40_enable,
		ht40_chan_busy,
		bw_mgr->bw_decision.ht40_scan_enable,
		__func__, __LINE__);
}

static void get_affected_chans(struct smart_bw_mgr *bw_mgr);
static void check_ht20_network(struct smart_bw_mgr *bw_mgr)
{
	ht40_bss_t *ht40_bss = NULL;
	int num = 0;

	g_last_connect_record.ht40p_weight =
	bw_mgr->chan_weight[1 - 1] +
	bw_mgr->chan_weight[5 - 1];
	g_last_connect_record.ht40m_weight =
	bw_mgr->chan_weight[7 - 1] +
	bw_mgr->chan_weight[11 - 1];

	for (num = 0; num < smart_bw_mgr_g.scan_results.ht40_bss_num; num++) {
		ht40_bss = &smart_bw_mgr_g.scan_results.ht40_bss[num];
		if (!ht40_bss)
			break;
		bw_mgr->ht2g_cur_bss.chan_idx = ht40_bss->chan_id;
		bw_mgr->ht2g_cur_bss.sec_ch_offset = ht40_bss->chan_offset;
		get_affected_chans(bw_mgr);
		process_scan_res(bw_mgr);
		check_ht40_chan_busy(bw_mgr);
	}
	smbw_err("smbw--ht40p_weight:%d,ht40m_weight:%d,exist_ht40_conflict:%d,ht40_chan_busy:%d",
		g_last_connect_record.ht40p_weight,
		g_last_connect_record.ht40m_weight,
		g_last_connect_record.exist_ht40_conflict,
		g_last_connect_record.ht40_chan_busy);
}

static void enable_channel_bonding_24ghz(bool enable);
static void write_connect_record()
{
	if (smart_bw_mgr_g.ht2g_cur_bss.ht40_support) {
		g_last_connect_record.ht40_enable =
			smart_bw_mgr_g.bw_decision.ht40_enable &&
			(smart_bw_mgr_g.ht2g_cur_bss.rssi_raw > -75);
	} else {
		g_last_connect_record.ht40_enable =
		!g_last_connect_record.ht40_intolerant &&
		!g_last_connect_record.exist_ht40_conflict &&
		!g_last_connect_record.ht40_chan_busy &&
		g_last_connect_record.ht40_bg_enable &&
		((g_last_connect_record.ht40p_weight <
		g_acs_threshold_cfg.acs_weight_thre) &&
		(g_last_connect_record.ht40m_weight <
		g_acs_threshold_cfg.acs_weight_thre));
	}
	g_last_connect_record.time_stamp = ktime_get();
	g_last_connect_record.updated = 1;
	smart_bw_mgr_g.connect_record_expire = 0;
	if (g_last_connect_record.ht40_enable)
		enable_channel_bonding_24ghz(true);
	else
		enable_channel_bonding_24ghz(false);

	smbw_err("smbw--connect_record.ht40_enable:%d, @%s, %d",
		g_last_connect_record.ht40_enable,
		__func__, __LINE__);
}

static void update_rssi_raw(struct ht2g_bss_info *cur_bss)
{
	ht40_bss_t *ht40_bss = NULL;
	int num = 0;

	for (num = 0; num < smart_bw_mgr_g.scan_results.ht40_bss_num; num++) {
		ht40_bss = &smart_bw_mgr_g.scan_results.ht40_bss[num];
		if (!ht40_bss)
			continue;
		if (memcmp(&ht40_bss->bssid[0], &cur_bss->bssid[0], 6) == 0) {
			cur_bss->rssi_raw = ht40_bss->rssi_raw;
			smbw_err("smbw--cur_bss->rssi_raw:%d, @%s, %d",
				cur_bss->rssi_raw,
				__func__, __LINE__);
		}
	}
}

static void make_ht40_decision(struct smart_bw_mgr *bw_mgr)
{
	make_scan_bw_decision(bw_mgr);
	cal_chl_weight_ht40_24_g(bw_mgr);
	make_chan_bw_decision(bw_mgr);
}

static void handle_chan_stats_work(struct work_struct *work)
{
	struct smart_bw_mgr *bw_mgr = NULL;
	struct wlan_objmgr_pdev *pdev = NULL;
	qdf_list_t *scan_res_list = NULL;
	tAcsSpectChInfo *pSpectCh = NULL;
	int i = 0;

	bw_mgr = &smart_bw_mgr_g;
	pdev = bw_mgr->pdev;
	if (!pdev) {
		smbw_err("smbw--pdev NULL pointer!!, @%s, %d",
			__func__, __LINE__);
		bw_mgr->chan_stats_wake = 0;
	}

	RETRY:
	wait_event(bw_mgr->chan_stats_waitq, bw_mgr->chan_stats_wake ||
		bw_mgr->func_exit);
	bw_mgr->chan_stats_wake = 0;
	if (unlikely(bw_mgr->func_exit)) {
		clear_SpectInfoParams(bw_mgr);
		clear_chan_stats(bw_mgr);
		smbw_err("smbw--exit work, @%s, %d",
			__func__, __LINE__);
		return;
	}

	scan_res_list = ucfg_scan_get_result(pdev, NULL);
	if (!scan_res_list) {
		smbw_err("smbw--get scan result failed, @%s, %d",
			__func__, __LINE__);
		clear_chan_stats(bw_mgr);
		goto RETRY;
	} else {
		smbw_debug("smbw--num of scan results is %d, @%s, %d",
			qdf_list_size(scan_res_list), __func__, __LINE__);
	}

	handle_scan_res(scan_res_list);
	if (bw_mgr->scan_results.ht40_bss_num > 0) {
		cal_spect_weight(&bw_mgr->SpectInfoParams,
			bw_mgr, scan_res_list);
		pSpectCh = &bw_mgr->SpectInfoParams.pSpectCh[0];
		qdf_spin_lock(&bw_mgr->chan_weight_lock);
		for (i = 0; i < 13; i++) {
			bw_mgr->chan_weight[i] = pSpectCh->weight;
			pSpectCh++;
		}
		bw_mgr->chan_weight_timestamp = ktime_get();
		qdf_spin_unlock(&bw_mgr->chan_weight_lock);
		if (bw_mgr->ht2g_cur_bss.ht40_support) {
			update_rssi_raw(&bw_mgr->ht2g_cur_bss);
			process_scan_res(bw_mgr);
			make_ht40_decision(bw_mgr);
			if (!bw_mgr->ht2g_connected)
				write_connect_record();
		} else {
			check_ht20_network(bw_mgr);
			write_connect_record();
		}
	} else {
		smbw_err("smbw--ht40_bss_num is 0 !!, @%s, %d",
			__func__, __LINE__);
	}
	clear_SpectInfoParams(bw_mgr);
	clear_chan_stats(bw_mgr);
	ucfg_scan_purge_results(scan_res_list);

	goto RETRY;
}

static int init_buffer_desc(struct smart_bw_mgr *bw_mgr)
{
	buffer_desc_t *buffer_desc = NULL;

	buffer_desc = &bw_mgr->ll_stats_metric.buffer_desc;
	buffer_desc->head_index = 0;
	buffer_desc->tail_index = 0;
	buffer_desc->cnt = 0;

	return 0;
}

void set_snr_monitor_enable(struct hdd_context *hdd_ctx, int enable)
{
	struct wlan_scan_obj *scan_obj;

	if (!hdd_ctx || !hdd_ctx->psoc) {
		smbw_err("smbw--psoc is NULL @%s, %d", __func__, __LINE__);
		return;
	}
	scan_obj = wlan_psoc_get_scan_obj(hdd_ctx->psoc);
	if (!scan_obj) {
		smbw_err("smbw--scan_obj is NULL @%s, %d",
			__func__, __LINE__);
		return;
	}
	scan_obj->scan_def.scan_f_chan_stat_evnt = enable;
}

static void init_manage_bss_list()
{
	qdf_list_t *white_bss_list = NULL;
	qdf_list_t *black_bss_list = NULL;

	white_bss_list = &g_manage_bss_list.white_bss_list_head;
	black_bss_list = &g_manage_bss_list.black_bss_list_head;
	qdf_init_list_head(&white_bss_list->anchor);
	qdf_init_list_head(&black_bss_list->anchor);
	white_bss_list->count = 0;
	black_bss_list->count = 0;
}

static void deinit_manage_bss_list()
{
	ht40_bss_node_t *cursor_node, *next_ode;
	qdf_list_t *white_bss_list = NULL;
	qdf_list_t *black_bss_list = NULL;

	white_bss_list = &g_manage_bss_list.white_bss_list_head;
	black_bss_list = &g_manage_bss_list.black_bss_list_head;

	if(!qdf_list_empty(white_bss_list)) {
		qdf_list_for_each_del(white_bss_list, cursor_node, next_ode, node) {
			qdf_list_remove_node(white_bss_list, &cursor_node->node);
			qdf_mem_free(cursor_node);
		}
	}

	if(!qdf_list_empty(black_bss_list)) {
		qdf_list_for_each_del(black_bss_list, cursor_node, next_ode, node) {
			qdf_list_remove_node(black_bss_list, &cursor_node->node);
			qdf_mem_free(cursor_node);
		}
	}
	white_bss_list->count = 0;
	black_bss_list->count = 0;
}

static void add_black_bss_list(struct smart_bw_mgr *bw_mgr)
{
	ht40_bss_node_t *bss_node = NULL;
	qdf_list_t *black_bss_list = NULL;

	black_bss_list = &g_manage_bss_list.black_bss_list_head;
	bss_node = qdf_mem_malloc_atomic(sizeof(ht40_bss_node_t));
	if (!bss_node) {
		smbw_err("smbw--mem_malloc failed!!, @%s, %d",
			__func__, __LINE__);
		return;
	}
	memcpy(&bss_node->bssid[0], &bw_mgr->ht2g_cur_bss.bssid[0], ETH_ALEN);
	bss_node->chan_idx = bw_mgr->ht2g_cur_bss.chan_idx;
	bss_node->sec_ch_offset = bw_mgr->ht2g_cur_bss.sec_ch_offset;
	bss_node->avg_rx_mcs = bw_mgr->ll_stats_metric.avg_rx_mcs;
	bss_node->avg_weight = bw_mgr->parent_cur_weight;
	bss_node->rssi_raw = bw_mgr->ll_stats_metric.avg_rssi;
	qdf_list_insert_front(black_bss_list, &bss_node->node);
}

static int check_black_bss_list(unsigned char *bssid)
{
	qdf_list_t *black_bss_list = NULL;
	ht40_bss_node_t *cur_node = NULL;
	ht40_bss_node_t *next_node = NULL;

	black_bss_list = &g_manage_bss_list.black_bss_list_head;
	if(qdf_list_empty(black_bss_list)) {
		smbw_debug("smbw--black_bss_list empty !!, @%s, %d",
			__func__, __LINE__);
		return 0;
	}

	qdf_list_peek_front(black_bss_list,(qdf_list_node_t **) &cur_node);
	if (!cur_node) {
		smbw_err("smbw--cur_node is NULL @%s, %d",
			__func__, __LINE__);
		return 0;
	}
	while (cur_node) {
		if (memcmp(bssid, &cur_node->bssid[0], 6) == 0) {
			smbw_err("smbw--bss lie in black_bss_list!,@%s, %d",
				__func__, __LINE__);
			return 1;
		}
		qdf_list_peek_next(black_bss_list,
			(qdf_list_node_t *) cur_node,
			(qdf_list_node_t **) &next_node);
		cur_node = next_node;
		next_node = NULL;
	}
	return 0;
}

static void clear_stats_metric(stats_metric_t *stats_metric)
{
	qdf_mem_zero(stats_metric, sizeof(stats_metric_t));
}

int smbw_ll_process_radio_stats(tSirLLStatsResults *link_stats,
	struct wifi_radio_stats *rs_results, 
	wmi_channel_stats *channel_stats)
{
	wmi_channel_stats *chan_stats_node;
	unsigned int num_channels;
	unsigned char chan_idx = 0;
	stats_metric_t *stats_metric = NULL;
	unsigned int tail_index = 0;
	ll_stats_t *ll_stats = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;
	int count;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit)) {
		smbw_debug("smbw--feature disabled or func_exit! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (!link_stats || !rs_results) {
		smbw_err("smbw--NULL pointer! @%s, %d", __func__, __LINE__);
		return -1;
	}
	bw_mgr = &smart_bw_mgr_g;
	ll_stats = &bw_mgr->ll_stats;

	if (!bw_mgr->ht40_connected || bw_mgr->vdev_id != link_stats->ifaceId) {
		smbw_debug("smbw--no process radio_stats for no2g bss,@%s,%d",
		__func__, __LINE__);
		return -1;
	}

	if (channel_stats) {
		chan_stats_node = channel_stats;
		if (chan_stats_node->center_freq < 2412 ||
			chan_stats_node->center_freq > 2484) {
			smbw_debug("smbw--channel not match @%s, %d",
				__func__, __LINE__);
			return -1;
		}
	} else {
		smbw_err("smbw--channel_stats is NULL! @%s, %d",
			__func__, __LINE__);
		return -1;
	}
	bw_mgr->radio_id_2g = rs_results->radio;

	ll_stats->cur_time = ktime_get();
	if (ll_stats->cal_parent_time == 0)
		ll_stats->cal_parent_time = ll_stats->cur_time;
	ll_stats->tsf_delta =
		ktime_to_ms(ll_stats->cur_time) -
		ktime_to_ms(ll_stats->cal_parent_time);

	num_channels = rs_results->num_channels;
	if (ll_stats->tsf_delta < 900 || num_channels < 2) {
		smbw_debug("smbw--tsf_delta:%d ,num_chan not match,@%s, %d",
			ll_stats->tsf_delta, __func__, __LINE__);
		ll_stats->cal_parent_time = ll_stats->cur_time;
		return -1;
	}

	tail_index = bw_mgr->ll_stats_metric.buffer_desc.tail_index;
	smbw_debug("smbw--tail_index:%d, num_channels:%d, @%s, %d",
		tail_index, num_channels, __func__, __LINE__);
	stats_metric = &bw_mgr->ll_stats_metric.stats_metric[tail_index];
	if (!stats_metric) {
		smbw_debug("smbw--NULL pointer! @%s, %d", __func__, __LINE__);
		return -1;
	}
	clear_stats_metric(stats_metric);

	for (count = 0; count < num_channels && count < 32; count++) {
		chan_idx = freq_to_chan_2g(chan_stats_node->center_freq);
		ll_stats->cca_busy_state.chan_stat[count].radio_awake_time =
			chan_stats_node->radio_awake_time;
		ll_stats->cca_busy_state.chan_stat[count].cca_busy_time =
		chan_stats_node->cca_busy_time;
		ll_stats->cca_busy_state.chan_stat[count].chan_id = chan_idx;
		chan_stats_node++;
	}
	ll_stats->cca_busy_state.stat_num = count;
	ll_stats->cca_busy_state.time_stamp = ll_stats->cur_time;

	ll_stats->sample_state.radio_sampled = 1;
	ll_stats->cal_parent_time = ll_stats->cur_time;

	return 0;
}

int smbw_ll_process_iface_stats(wmi_iface_link_stats *link_stats,
	wmi_wmm_ac_stats *iface_ac_stats,
	unsigned int vdev_id)
{
	stats_metric_t *stats_metric = NULL;
	wmi_wmm_ac_stats *ac_stat = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;
	unsigned long total_data_num = 0;
	unsigned char num = 0;
	unsigned int tail_index = 0;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit)) {
		smbw_debug("smbw--feature disabled or func_exit! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (!link_stats || !iface_ac_stats) {
		smbw_err("smbw--NULL pointer @%s, %d",
			__func__, __LINE__);
		return -1;
	}
	bw_mgr = &smart_bw_mgr_g;
	if (!bw_mgr->ht40_connected || bw_mgr->vdev_id != vdev_id) {
		smbw_debug("smbw--ht2g not connected @%s, %d",
			__func__, __LINE__);
		return -1;
	}
	if (bw_mgr->ll_stats.tsf_delta < 900) {
		smbw_debug("smbw--tsf_delta < 900 @%s, %d",
			__func__, __LINE__);
		return -1;
	} 
 
	tail_index = bw_mgr->ll_stats_metric.buffer_desc.tail_index;
	smbw_debug("smbw--tail_index:%d, @%s, %d", tail_index,
		__func__, __LINE__);
	stats_metric = &bw_mgr->ll_stats_metric.stats_metric[tail_index];
	if (iface_ac_stats) {
		ac_stat = iface_ac_stats;
		for (num = 0; num < link_stats->num_ac; num++) {
			total_data_num +=
			(ac_stat->tx_mpdu + ac_stat->rx_mpdu +
			ac_stat->tx_ampdu * 3 + ac_stat->rx_ampdu * 3 +
			ac_stat->tx_mcast + ac_stat->rx_mcast);
			ac_stat++;
		}

		if (total_data_num > 0 && total_data_num > bw_mgr->ll_stats.parent_data_num) {
			stats_metric->data_flow =
			(total_data_num - bw_mgr->ll_stats.parent_data_num) /
			g_acs_threshold_cfg.sample_interval;
			bw_mgr->ll_stats.parent_data_num = total_data_num;
			smbw_debug("smbw--total_data_num:%d, data_flow:%d, @%s, %d",
			total_data_num, stats_metric->data_flow,
			__func__, __LINE__);
		} else {
			bw_mgr->ll_stats.parent_data_num = 0;
			smbw_debug("smbw--ERROR: total_data_num:%d,@%s, %d",
				total_data_num,__func__, __LINE__);
		}
	}

	bw_mgr->ll_stats.sample_state.iface_sampled = 1;
	return 0;
}

static void cal_cca_busy_score(struct smart_bw_mgr *bw_mgr)
{
	unsigned char chan_idx = 0, num = 0, stat_num = 0;
	unsigned char af_chan_start = 0, af_chan_end = 0;
	stats_metric_t *stats_metric = NULL;
	unsigned int tail_index = 0, temp_cca_ratio = 0, temp_cca_score = 0;
	ll_stats_t *ll_stats = NULL;

	tail_index = bw_mgr->ll_stats_metric.buffer_desc.tail_index;
	ll_stats = &bw_mgr->ll_stats;
	smbw_debug("smbw--tail_index:%d, @%s, %d",
		tail_index,__func__, __LINE__);
	stats_metric = &bw_mgr->ll_stats_metric.stats_metric[tail_index];
	if (!stats_metric) {
		smbw_err("smbw--NULL pointer! @%s, %d", __func__, __LINE__);
		return;
	}
	stats_metric->cca_busy_ratio.time_stamp =
		ll_stats->cca_busy_state.time_stamp;
	stat_num = ll_stats->cca_busy_state.stat_num;
	for (num = 0; num < (stat_num - 1); num++) {
		chan_idx = ll_stats->cca_busy_state.chan_stat[num].chan_id;
		if (ll_stats->cca_busy_state.chan_stat[num].radio_awake_time <= 0 ||
			ll_stats->cca_busy_state.chan_stat[num].cca_busy_time <= 0 ||
			ll_stats->cca_busy_state.chan_stat[num].cca_busy_time >
			ll_stats->cca_busy_state.chan_stat[num].radio_awake_time) {
			smbw_debug("smbw--radio_awake_time <= 0! @%s, %d",
				__func__, __LINE__);
			continue;
		}
		stats_metric->stat_num_chan[chan_idx] += 1;
		temp_cca_ratio =
		(ll_stats->cca_busy_state.chan_stat[num].cca_busy_time * 100) /
		ll_stats->cca_busy_state.chan_stat[num].radio_awake_time;
		stats_metric->cca_busy_ratio.cca_busy_ratio[chan_idx] =
			temp_cca_ratio;

		smbw_debug("smbw--cca_busy_ratio:%d, cca_busy_time:%d, radio_awake_time:%d, @%s, %d",
		stats_metric->cca_busy_ratio.cca_busy_ratio[chan_idx],
		ll_stats->cca_busy_state.chan_stat[num].cca_busy_time,
		ll_stats->cca_busy_state.chan_stat[num].radio_awake_time,
		__func__, __LINE__);
	}

	if (bw_mgr->ht40_connected) {
		stats_metric->cur_cca_busy_score.time_stamp =
			stats_metric->cca_busy_ratio.time_stamp;
		af_chan_start = bw_mgr->ht2g_cur_bss.af_chan_start;
		af_chan_end = bw_mgr->ht2g_cur_bss.af_chan_end;
		smbw_debug("smbw--af_chan_start:%d, af_chan_end:%d, @%s, %d",
		af_chan_start, af_chan_end,
		__func__, __LINE__);

		for (chan_idx = af_chan_start; chan_idx <= af_chan_end; chan_idx++) {
			temp_cca_score =
			stats_metric->cca_busy_ratio.cca_busy_ratio[chan_idx];
			if (temp_cca_score >
			stats_metric->cur_cca_busy_score.cca_busy_score)
				stats_metric->cur_cca_busy_score.cca_busy_score =
				temp_cca_score;
			stats_metric->cur_ht40_stat_num +=
			stats_metric->stat_num_chan[chan_idx];
		}
		smbw_debug("smbw--cur_cca_busy_score:%d,num:%d,@%s, %d",
			stats_metric->cur_cca_busy_score.cca_busy_score,
			stats_metric->cur_ht40_stat_num,
			__func__, __LINE__);
	}
}

unsigned char sample_weight[6] = {1, 2, 3, 6, 8, 12};

static void cal_avg_ll_score(struct smart_bw_mgr *bw_mgr)
{
	stats_metric_t *stats_metric[6], *stats_metric_tail, *tmp_p = NULL;
	unsigned int tmp_id = 0, head_id = 0, tail_id = 0, cnt = 0, num = 0;
	unsigned int cur_cca_busy_score = 0, data_flow_score = 0;
	int total_rssi = 0;

	head_id = bw_mgr->ll_stats_metric.buffer_desc.head_index;
	tail_id = bw_mgr->ll_stats_metric.buffer_desc.tail_index;
	cnt = bw_mgr->ll_stats_metric.buffer_desc.cnt;
	smbw_debug("smbw--head_id:%d, tail_id:%d, cnt:%d, @%s, %d",
		head_id, tail_id, cnt, __func__, __LINE__);

	tmp_id = head_id;
	while (tmp_id <= 5 && cnt--) {
		stats_metric[num++] =
			&bw_mgr->ll_stats_metric.stats_metric[tmp_id++];
	}
	if (tmp_id > 5 && cnt > 0) {
		tmp_id = 0;
		while (tmp_id <= 5 && cnt--) {
			stats_metric[num++] =
			&bw_mgr->ll_stats_metric.stats_metric[tmp_id++];
		}
	}
	cnt = bw_mgr->ll_stats_metric.buffer_desc.cnt;
	if (cnt == 6) {
		for (num = 0; num < cnt; num++) {
			tmp_p = stats_metric[num];
			if (!tmp_p) {
				smbw_debug("smbw--NULL pointer!! @%s, %d",
					__func__, __LINE__);
				return;
			}
			cur_cca_busy_score +=
				tmp_p->cur_cca_busy_score.cca_busy_score *
				sample_weight[num];
			data_flow_score +=
				tmp_p->data_flow * sample_weight[num];
			total_rssi +=
				tmp_p->sta_rate_info.rssi * sample_weight[num];
			smbw_debug("smbw--cur_cca_bs:%lu,data_flow_score:%lu,@%s, %d",
				cur_cca_busy_score,
				data_flow_score,
				__func__, __LINE__);
		}
		qdf_spin_lock(&bw_mgr->ll_stats_metric.ll_metric_lock);
		bw_mgr->ll_stats_metric.avg_cur_cca_busy_score = cur_cca_busy_score / 32;
		bw_mgr->ll_stats_metric.avg_data_flow_score = data_flow_score / 32;
		bw_mgr->ll_stats_metric.avg_rssi = total_rssi / 32;
		qdf_spin_unlock(&bw_mgr->ll_stats_metric.ll_metric_lock);
	} else {
		stats_metric_tail = &bw_mgr->ll_stats_metric.stats_metric[tail_id];
		if (!stats_metric_tail) {
			smbw_err("smbw--NULL pointer!!, @%s, %d",
				__func__, __LINE__);
			return;
		}
		qdf_spin_lock(&bw_mgr->ll_stats_metric.ll_metric_lock);
		bw_mgr->ll_stats_metric.avg_cur_cca_busy_score =
			stats_metric_tail->cur_cca_busy_score.cca_busy_score;
		bw_mgr->ll_stats_metric.avg_data_flow_score =
			stats_metric_tail->data_flow;
		bw_mgr->ll_stats_metric.avg_rssi =
			stats_metric_tail->sta_rate_info.rssi;
		qdf_spin_unlock(&bw_mgr->ll_stats_metric.ll_metric_lock);
	}
    
	if (bw_mgr->ll_stats_metric.adaptive_threshold.sample_num % STATS_SAMPLE_NUM == 0) {
		bw_mgr->ll_stats_metric.avg_rx_mcs =
		bw_mgr->ll_stats_metric.adaptive_threshold.total_rx_mcs / STATS_SAMPLE_NUM;
		bw_mgr->ll_stats_metric.adaptive_avg_weight =
		bw_mgr->ll_stats_metric.adaptive_threshold.total_weight / STATS_SAMPLE_NUM;
		bw_mgr->ll_stats_metric.adaptive_threshold.sample_num = 0;
		bw_mgr->ll_stats_metric.adaptive_threshold.total_rx_mcs = 0;
		bw_mgr->ll_stats_metric.adaptive_threshold.total_weight = 0;
		bw_mgr->ht2g_cur_bss.avg_weight =
			bw_mgr->ll_stats_metric.adaptive_avg_weight;
		if (bw_mgr->ll_stats_metric.avg_rx_mcs <= 2 &&
			bw_mgr->ll_stats_metric.avg_rx_mcs != 0 &&
			bw_mgr->ll_stats_metric.avg_rssi > -72) {
			bw_mgr->ht2g_cur_bss.need_post_lq_log = 1;
			bw_mgr->ht2g_cur_bss.need_post_chan_log = 1;
			smbw_err("smbw--avg_rx_mcs <= 2,need post_log!! @%s, %d",
				__func__, __LINE__);
		}
		if (bw_mgr->ll_stats_metric.avg_rx_mcs <= 2 ||
		bw_mgr->ll_stats_metric.adaptive_avg_weight >
		g_acs_threshold_cfg.acs_weight_thre ||
		(ABS(bw_mgr->ll_stats_metric.adaptive_avg_weight -
		g_acs_threshold_cfg.acs_weight_thre) < 200 &&
		bw_mgr->ll_stats_metric.avg_rx_mcs >= 8)) {
			bw_mgr->ht2g_cur_bss.weight_adjuste_enable = 1;
		}
		smbw_err("smbw--avg_rx_mcs:%d, adaptive_avg_weight:%d",
			bw_mgr->ll_stats_metric.avg_rx_mcs,
			bw_mgr->ll_stats_metric.adaptive_avg_weight);
	}
	smbw_debug("smbw--avg_cur_cca_bs:%d,avg_dflow:%d,avg_rssi:%d",
		bw_mgr->ll_stats_metric.avg_cur_cca_busy_score,
		bw_mgr->ll_stats_metric.avg_data_flow_score,
		bw_mgr->ll_stats_metric.avg_rssi);
}

static void update_buffer_desc(struct smart_bw_mgr *bw_mgr)
{
	unsigned int head_id = 0, tail_id = 0, cnt = 0;
	buffer_desc_t *buffer_desc = NULL;

	buffer_desc = &bw_mgr->ll_stats_metric.buffer_desc;
	head_id = buffer_desc->head_index;
	tail_id = buffer_desc->tail_index;
	cnt = buffer_desc->cnt;

	qdf_spin_lock(&buffer_desc->stat_buf_lock);
	if (tail_id == 5) {
		buffer_desc->tail_index = 0;
		if (buffer_desc->tail_index == head_id)
			buffer_desc->head_index += 1;
	} else if (tail_id + 1 == head_id) {
		buffer_desc->tail_index += 1;
		if (head_id == 5)
			buffer_desc->head_index = 0;
		else
			buffer_desc->head_index += 1;
	} else {
		buffer_desc->tail_index += 1;
	}
	bw_mgr->ll_stats_metric.buffer_desc.cnt += 1;
	if (bw_mgr->ll_stats_metric.buffer_desc.cnt > 6)
		bw_mgr->ll_stats_metric.buffer_desc.cnt = 6;
	qdf_spin_unlock(&buffer_desc->stat_buf_lock);
	if (head_id > 5 || tail_id > 5)
		smbw_debug("smbw--head_id > 5 || tail_id > 5!!! @%s, %d",
			__func__, __LINE__);
}

static void adjust_acs_threshold(struct smart_bw_mgr *bw_mgr)
{
	unsigned int avg_rx_mcs = 0, last_acs_thre = 0, avg_weight = 0;
	unsigned char tx_nss = 0, rx_nss = 0, nss = 0;
	int avg_rssi;

	avg_rssi = bw_mgr->ll_stats_metric.avg_rssi;
	avg_rx_mcs = bw_mgr->ll_stats_metric.avg_rx_mcs;
	avg_weight = bw_mgr->ll_stats_metric.adaptive_avg_weight;
	if (avg_rx_mcs == 0 || avg_rx_mcs > 32 || avg_rssi < -70) {
		smbw_debug("smbw--invalid !!, just return@%s, %d", __func__, __LINE__);
		return;
	}

#if 1
	if (bw_mgr->ht2g_cur_bss.weight_adjuste_enable == 0 ||
		bw_mgr->ht2g_cur_bss.weight_adjusted == 1) {
		smbw_debug("smbw--weight_adjuste not enable, @%s, %d", __func__, __LINE__);
		return;
	}
#endif
	tx_nss = bw_mgr->ll_stats_metric.tx_nss;
	rx_nss = bw_mgr->ll_stats_metric.rx_nss;
	nss = tx_nss < rx_nss ? tx_nss : rx_nss;
	last_acs_thre = g_acs_threshold_cfg.acs_weight_thre;
	switch(nss) {
	case 1:
	if (avg_rx_mcs <= g_acs_threshold_cfg.bad_mcs &&
		avg_weight < g_acs_threshold_cfg.acs_weight_thre) {
		g_acs_threshold_cfg.acs_weight_thre -= 100;
		bw_mgr->ht2g_cur_bss.weight_adjusted = 1;
	} else if (avg_rx_mcs >= g_acs_threshold_cfg.good_mcs &&
		(avg_weight >= g_acs_threshold_cfg.acs_weight_thre ||
		g_acs_threshold_cfg.acs_weight_thre - avg_weight < 100)) {
		g_acs_threshold_cfg.acs_weight_thre += 100;
		bw_mgr->ht2g_cur_bss.weight_adjusted = 1;
	}
        	break;
	case 2:
	if (avg_rx_mcs <= g_acs_threshold_cfg.bad_mcs &&
		avg_weight < g_acs_threshold_cfg.acs_weight_thre) {
		g_acs_threshold_cfg.acs_weight_thre -= 100;
		bw_mgr->ht2g_cur_bss.weight_adjusted = 1;
	} else if (avg_rx_mcs >= g_acs_threshold_cfg.good_mcs &&
		(avg_weight >= g_acs_threshold_cfg.acs_weight_thre ||
		g_acs_threshold_cfg.acs_weight_thre - avg_weight < 100)) {
		g_acs_threshold_cfg.acs_weight_thre += 100;
		bw_mgr->ht2g_cur_bss.weight_adjusted = 1;
        }
		break;
	default:
		break;
	}
	bw_mgr->ht2g_cur_bss.weight_adjuste_enable = 0;
	if (g_acs_threshold_cfg.acs_weight_thre < 34500)
		g_acs_threshold_cfg.acs_weight_thre = 34500;
	else if (g_acs_threshold_cfg.acs_weight_thre > 38000)
		g_acs_threshold_cfg.acs_weight_thre = 38000;
	smbw_debug("smbw--avg_rx_mcs:%d,avg_weight:%d,rx_nss:%d,last_thre:%d,cur_thre:%d,@%s,%d",
		avg_rx_mcs,avg_weight,rx_nss,last_acs_thre,
		g_acs_threshold_cfg.acs_weight_thre,__func__, __LINE__);
}

static void make_ll_bw_decision(struct smart_bw_mgr *bw_mgr)
{
	qdf_spin_lock(&bw_mgr->bw_decision.bw_decision_lock);

	if (bw_mgr->ht40_connected) {
		if ((bw_mgr->ll_stats_metric.avg_cur_cca_busy_score >
		g_acs_threshold_cfg.ll_com_score_thre &&
		bw_mgr->ll_stats_metric.avg_data_flow_score < 5000) ||
		bw_mgr->ll_stats_metric.avg_rx_mcs < 2)
			bw_mgr->bw_decision.ht40_ll_enable = 0;
		else
			bw_mgr->bw_decision.ht40_ll_enable = 1;
	}

	if (bw_mgr->ll_stats_metric.avg_rx_mcs <= 1 &&
		bw_mgr->ll_stats_metric.avg_rssi > -65 &&
		bw_mgr->parent_cur_weight <
		g_acs_threshold_cfg.acs_weight_thre &&
		bw_mgr->ll_stats_metric.avg_data_flow_score > 160 &&
		bw_mgr->ll_stats_metric.avg_rx_mcs != 0) {
		add_black_bss_list(bw_mgr);
		smbw_err("smbw--add_black_list-avg_rx_mcs:%d,avg_rssi:%d,avg_weight:%d,data_flow:%d,@%s,%d",
			bw_mgr->ll_stats_metric.avg_rx_mcs,
			bw_mgr->ll_stats_metric.avg_rssi,
			bw_mgr->parent_cur_weight,
			bw_mgr->ll_stats_metric.avg_data_flow_score,
			__func__, __LINE__);
	}
#if 0
	if (bw_mgr->bw_decision.ht40_ll_enable)
		bw_mgr->bw_decision.ht40_enable = 1;
	else
		bw_mgr->bw_decision.ht40_enable = 0;
#endif
	qdf_spin_unlock(&bw_mgr->bw_decision.bw_decision_lock);
	smbw_debug("smbw--ht40_ll_enable:%d,@%s,%d",
		bw_mgr->bw_decision.ht40_ll_enable,
		__func__, __LINE__);
}

static void clear_ll_stats(struct smart_bw_mgr *bw_mgr)
{
	bw_mgr->ll_stats.cca_busy_state.stat_num = 0;
	bw_mgr->ll_stats_metric.avg_cur_cca_busy_score = 0;
	bw_mgr->ll_stats_metric.avg_data_flow_score = 0;
}

int get_sta_stats(struct hdd_adapter *adapter)
{
	stats_metric_t *stats_metric = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;
	unsigned int tail_index = 0;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit || !adapter)) {
		smbw_debug("smbw--feature disabled or func_exit! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	bw_mgr = &smart_bw_mgr_g;
	if (!bw_mgr->ht40_connected) {
		smbw_debug("smbw--ht40 not connected @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	tail_index = bw_mgr->ll_stats_metric.buffer_desc.tail_index;
	stats_metric = &bw_mgr->ll_stats_metric.stats_metric[tail_index];

	stats_metric->sta_rate_info.rssi =
		adapter->hdd_stats.summary_stat.rssi;
	stats_metric->sta_rate_info.snr =
		adapter->hdd_stats.summary_stat.snr;
	stats_metric->sta_rate_info.tx_mcs_index =
		adapter->hdd_stats.class_a_stat.tx_mcs_index;
	stats_metric->sta_rate_info.rx_mcs_index =
		adapter->hdd_stats.class_a_stat.rx_mcs_index;
	stats_metric->sta_rate_info.tx_gi =
		adapter->hdd_stats.class_a_stat.tx_gi;
	stats_metric->sta_rate_info.rx_gi =
		adapter->hdd_stats.class_a_stat.rx_gi;
	stats_metric->sta_rate_info.tx_nss =
		adapter->hdd_stats.class_a_stat.tx_nss;
	stats_metric->sta_rate_info.rx_nss =
		adapter->hdd_stats.class_a_stat.rx_nss;
	bw_mgr->ll_stats_metric.tx_nss =
		adapter->hdd_stats.class_a_stat.tx_nss;
	bw_mgr->ll_stats_metric.rx_nss =
		adapter->hdd_stats.class_a_stat.rx_nss;
	bw_mgr->ll_stats_metric.tx_rx_rate_flags =
		adapter->hdd_stats.class_a_stat.tx_rx_rate_flags;

	bw_mgr->ll_stats.sample_state.sta_stats_sampled = 1;
	if (bw_mgr->ll_stats.sample_state.radio_sampled &&
		bw_mgr->ll_stats.sample_state.iface_sampled) {
		if (stats_metric->sta_rate_info.rx_mcs_index > 31 ||
			stats_metric->sta_rate_info.rx_mcs_index == 0) {
			smbw_debug("smbw--MCS Error! @%s, %d", __func__, __LINE__);
			stats_metric->sta_rate_info.tx_mcs_index = 255;
			stats_metric->sta_rate_info.rx_mcs_index = 255;
		} else {
			bw_mgr->ll_stats_metric.adaptive_threshold.sample_num += 1;
			bw_mgr->ll_stats_metric.adaptive_threshold.total_rx_mcs +=
				stats_metric->sta_rate_info.rx_mcs_index;
			bw_mgr->ll_stats_metric.adaptive_threshold.total_weight +=
				bw_mgr->cur_weight;
			smbw_debug("smbw--sample_num:%d, rx_mcs_index:%d, @%s, %d",
				bw_mgr->ll_stats_metric.adaptive_threshold.sample_num,
				stats_metric->sta_rate_info.rx_mcs_index,
				__func__, __LINE__);
		}

		bw_mgr->ll_stats.sample_state.radio_sampled = 0;
		bw_mgr->ll_stats.sample_state.iface_sampled = 0;
		bw_mgr->ll_stats.sample_state.sta_stats_sampled = 0;
		ll_stats_wake_up(bw_mgr);
		smbw_debug("smbw--queue_work:ll_work, cnt:%d,@%s, %d",
		bw_mgr->ll_stats_metric.buffer_desc.cnt, __func__, __LINE__);
		queue_work(bw_mgr->ll_stats_queue, &bw_mgr->ll_stats_work);
	}
	smbw_debug("smbw--txmcs:%d,rxmcs:%d,tx_gi:%d,rx_gi:%d,tx_nss:%d,rx_nss:%d,rssi:%d,rate_flags:%d",
		stats_metric->sta_rate_info.tx_mcs_index,
		stats_metric->sta_rate_info.rx_mcs_index,
		stats_metric->sta_rate_info.tx_gi,
		stats_metric->sta_rate_info.rx_gi,
		stats_metric->sta_rate_info.tx_nss,
		stats_metric->sta_rate_info.rx_nss,
		stats_metric->sta_rate_info.rssi,
		bw_mgr->ll_stats_metric.tx_rx_rate_flags);

	return 0;
}

static void handle_stats_work(struct work_struct *work)
{
	struct smart_bw_mgr *bw_mgr = NULL;

	bw_mgr = &smart_bw_mgr_g;

RETRY:
	wait_event(bw_mgr->ll_stats_waitq, bw_mgr->ll_stats_wake ||
		bw_mgr->func_exit);
	bw_mgr->ll_stats_wake = 0;
	if (unlikely(bw_mgr->func_exit)) {
		update_buffer_desc(bw_mgr);
		clear_ll_stats(bw_mgr);
		smbw_err("smbw--exit work, @%s, %d",
			__func__, __LINE__);
		return;
	}
	cal_cca_busy_score(bw_mgr);
	cal_avg_ll_score(bw_mgr);
	update_buffer_desc(bw_mgr);
	post_kevent_log(bw_mgr);
	adjust_acs_threshold(bw_mgr);
	make_ll_bw_decision(bw_mgr);
	clear_ll_stats(bw_mgr);
	goto RETRY;
}

static void get_affected_chans(struct smart_bw_mgr *bw_mgr)
{
	struct ht2g_bss_info *ht2g_cur_bss = NULL;
	unsigned char CHAN_END = 11;

	if (bw_mgr->chan_stats_cnt > CHAN_END)
		CHAN_END = bw_mgr->chan_stats_cnt;
	if (CHAN_END < 11)
		CHAN_END = 11;

	ht2g_cur_bss = &bw_mgr->ht2g_cur_bss;
	if (ht2g_cur_bss->chan_idx >= 1 && ht2g_cur_bss->chan_idx <= 4) {
		ht2g_cur_bss->sec_chan_idx = ht2g_cur_bss->chan_idx + 4;
		ht2g_cur_bss->af_chan_start = 1;
		ht2g_cur_bss->af_chan_end = ht2g_cur_bss->chan_idx + 7;
	} else if (ht2g_cur_bss->chan_idx >= 10 &&
		ht2g_cur_bss->chan_idx <= CHAN_END) {
		ht2g_cur_bss->sec_chan_idx = ht2g_cur_bss->chan_idx - 4;
		ht2g_cur_bss->af_chan_start = ht2g_cur_bss->chan_idx - 7;
		ht2g_cur_bss->af_chan_end = CHAN_END;
	} else {
		if (ht2g_cur_bss->sec_ch_offset == 1) {
			ht2g_cur_bss->sec_chan_idx = ht2g_cur_bss->chan_idx + 4;
			ht2g_cur_bss->af_chan_start = ht2g_cur_bss->chan_idx + 2 - 5;
			if (ht2g_cur_bss->chan_idx + 7 > CHAN_END)
				ht2g_cur_bss->af_chan_end = CHAN_END;
			else
				ht2g_cur_bss->af_chan_end = ht2g_cur_bss->chan_idx + 7;
			if (ht2g_cur_bss->sec_chan_idx > CHAN_END) {
				ht2g_cur_bss->sec_chan_idx = CHAN_END;
				bw_mgr->bw_decision.exist_ht40_conflict = 1;
				smbw_err("smbw--invalid AP for HT40, @%s, %d",
					__func__, __LINE__);
			}
		} else if (ht2g_cur_bss->sec_ch_offset == -1) {
			ht2g_cur_bss->sec_chan_idx = ht2g_cur_bss->chan_idx - 4;
			ht2g_cur_bss->af_chan_end = ht2g_cur_bss->chan_idx - 2 + 5;
			if (ht2g_cur_bss->af_chan_end > CHAN_END)
				ht2g_cur_bss->af_chan_end = CHAN_END;
			if (ht2g_cur_bss->chan_idx - 7 < 1)
			    ht2g_cur_bss->af_chan_start = 1;
			else
				ht2g_cur_bss->af_chan_start =
					ht2g_cur_bss->chan_idx - 7;
		}
	}
	smbw_debug("smbw--af_chan_start:%d, af_chan_end:%d, @%s, %d",
		ht2g_cur_bss->af_chan_start,
		ht2g_cur_bss->af_chan_end,
		__func__, __LINE__);
}

static void enable_channel_bonding_24ghz(bool enable)
{
	if (!smart_bw_mgr_g.mac) {
		smbw_err("smbw--smart_bw_mgr_g.mac is NULL!");
		return;
	}

	if (enable) {
#if 1
		smart_bw_mgr_g.mac->roam.configParam.channelBondingMode24GHz =
			eHT_CHANNEL_WIDTH_40MHZ;
		smart_bw_mgr_g.mac->mlme_cfg->ht_caps.ht_cap_info.short_gi_40_mhz = 1;
		smart_bw_mgr_g.mac->mlme_cfg->ht_caps.ht_cap_info.supported_channel_width_set =
			eHT_CHANNEL_WIDTH_40MHZ;
#endif
		ucfg_mlme_set_channel_bonding_24ghz(smart_bw_mgr_g.mac->psoc,
			eHT_CHANNEL_WIDTH_40MHZ);
	} else {
#if 1
		smart_bw_mgr_g.mac->roam.configParam.channelBondingMode24GHz =
			eHT_CHANNEL_WIDTH_20MHZ;
		smart_bw_mgr_g.mac->mlme_cfg->ht_caps.ht_cap_info.short_gi_40_mhz = 0;
		smart_bw_mgr_g.mac->mlme_cfg->ht_caps.ht_cap_info.supported_channel_width_set =
			eHT_CHANNEL_WIDTH_20MHZ;
#endif
		ucfg_mlme_set_channel_bonding_24ghz(smart_bw_mgr_g.mac->psoc,
			eHT_CHANNEL_WIDTH_20MHZ);
	}
}

static int check_last_record()
{
#define RECORD_EXPIRE 20000
	ktime_t cur_time;
	int ht40_enable = 0;

	cur_time = ktime_get();
	if ((ktime_to_ms(cur_time) -
	ktime_to_ms(g_last_connect_record.time_stamp)) > RECORD_EXPIRE) {
		qdf_mem_zero(&g_last_connect_record, sizeof(last_record_t));
		smart_bw_mgr_g.connect_record_expire = 1;
		smbw_err("smbw--decision stats expire @%s, %d",
			__func__, __LINE__);
		return -1;
	} else {
		ht40_enable =
		g_last_connect_record.ht40_enable;
		smbw_debug("smbw--ht40_enable:%d @%s, %d",
		ht40_enable, __func__, __LINE__);
		return ht40_enable;
	}

	return ht40_enable;
}

int check_ht40_bw_enable(struct mac_context *mac, tDot11fIEHTCaps *pDot11f,
                        struct pe_session *pe_session)
{
	unsigned char ht40_enable = 0, num = 0, found = 0;
	int last_stat = 0;
	ht40_bss_t *ht40_bss = NULL;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit || !mac || !pDot11f)) {
		smbw_debug("smbw--feature disabled! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (pe_session && (pe_session->curr_op_freq > WLAN_CHAN_14_FREQ  ||
		pe_session->curr_op_freq < WLAN_24_GHZ_BASE_FREQ ||
		pe_session->opmode != QDF_STA_MODE)) {
		smbw_debug("smbw--not 2.4g band! or opmode != QDF_STA_MODE@%s, %d",
		__func__, __LINE__);
		return -1;
	}
	smart_bw_mgr_g.mac = mac;
	if (!smart_bw_mgr_g.ht2g_connected) {
		if (!pe_session) {
			smbw_debug("smbw--pe_session is NULL! @%s, %d",
				__func__, __LINE__);
			if (!smart_bw_mgr_g.connect_record_expire &&
				g_last_connect_record.updated) {
				last_stat = check_last_record();
				if (last_stat >= 0)
					ht40_enable = last_stat;
			}
			smbw_debug("smbw--ht40_enable:%d, @%s, %d",ht40_enable,
				__func__, __LINE__);
		} else {
			if (!qdf_spin_trylock(&smart_bw_mgr_g.scan_stats_lock)) {
				smbw_err("smbw--spin_trylock failed, @%s, %d",
				__func__, __LINE__);
				return -1;
			}
			for (num = 0;
				num < smart_bw_mgr_g.scan_results.ht40_bss_num;
				num++) {
				ht40_bss =
				&smart_bw_mgr_g.scan_results.ht40_bss[num];
				if (!ht40_bss)
					break; 
				if (memcmp(&ht40_bss->bssid[0], (unsigned char*)&pe_session->bssId, 6) == 0) {
					if (ht40_bss->rssi_raw < -75) {
						smbw_err("smbw--rssi_raw < -75 @%s, %d",
						__func__, __LINE__);
						ht40_enable = 0;
						break;
					} else if (ht40_bss->chan_offset == 1) {
						smart_bw_mgr_g.ht2g_cur_bss.chan_idx =
							ht40_bss->chan_id;
						smart_bw_mgr_g.ht2g_cur_bss.sec_ch_offset = 1;
					} else if (ht40_bss->chan_offset == -1) {
						smart_bw_mgr_g.ht2g_cur_bss.chan_idx =
							ht40_bss->chan_id;
						smart_bw_mgr_g.ht2g_cur_bss.sec_ch_offset = -1;
					}
					smart_bw_mgr_g.ht2g_cur_bss.rssi_raw = ht40_bss->rssi_raw;
					found = 1;
					smart_bw_mgr_g.ht2g_cur_bss.ht40_support = 1;
					get_affected_chans(&smart_bw_mgr_g);
					process_scan_res(&smart_bw_mgr_g);
					break;
				}
			}
			qdf_spin_unlock(&smart_bw_mgr_g.scan_stats_lock);
			if (found != 1) {
				smbw_err("smbw--not found!! @%s, %d",
					__func__, __LINE__);
				ht40_enable = 0;
				smart_bw_mgr_g.ht2g_cur_bss.ht40_support = 0;
			} else {
				if (check_black_bss_list((unsigned char*)&pe_session->bssId)) {
					ht40_enable = 0;
				} else {
					make_ht40_decision(&smart_bw_mgr_g);
					ht40_enable =
					smart_bw_mgr_g.bw_decision.ht40_enable;
					smbw_err("smbw--ht40_enable:%d, cur_weight:%d, @%s, %d",
					ht40_enable, smart_bw_mgr_g.parent_cur_weight,
					__func__, __LINE__);
				}
			}
		}
	} else if (smart_bw_mgr_g.ht40_connected) {
		ht40_enable = 1;
	} else {
		ht40_enable = 0;
	}

	smbw_debug("smbw--ht40_enable:%d,  @%s, %d",
		ht40_enable, __func__, __LINE__);

	if (ht40_enable) {
		pDot11f->supportedChannelWidthSet = eHT_CHANNEL_WIDTH_40MHZ;
		pDot11f->shortGI40MHz = 1;
		enable_channel_bonding_24ghz(true);
	} else {
		pDot11f->supportedChannelWidthSet = eHT_CHANNEL_WIDTH_20MHZ;
		pDot11f->shortGI40MHz = 0;
		enable_channel_bonding_24ghz(false);
	}
	return 0;
}

int get_ht2g_connected_bss(struct hdd_adapter *adapter,
	struct csr_roam_info *roam_info)
{
	struct ht2g_bss_info *ht2g_cur_bss = NULL;
	struct smart_bw_mgr *bw_mgr = NULL;
	unsigned char chan_no  = 0;
	if (unlikely(FEATURE_DISABLE || !roam_info || !adapter)) {
		smbw_debug("smbw--feature disabled! @%s, %d",
			__func__, __LINE__);
		return -1;
	}
	if (adapter->device_mode != QDF_STA_MODE) {
		smbw_err("smbw--device_mode != QDF_STA_MODE! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	chan_no = freq_to_chan_2g(roam_info->bss_desc->chan_freq);
	if (chan_no > 14 || chan_no < 1) {
		smbw_debug("smbw--chan not match @%s, %d",
			__func__, __LINE__);
		smart_bw_mgr_g.ht2g_cur_bss.ht40_support = 0;
		return -1;
	}
	bw_mgr = &smart_bw_mgr_g;

	bw_mgr->adapter = adapter;
	bw_mgr->vdev_id = adapter->vdev_id;
	ht2g_cur_bss = &bw_mgr->ht2g_cur_bss;
	memcpy(&ht2g_cur_bss->bssid[0], &roam_info->bssid.bytes[0], ETH_ALEN);
	ht2g_cur_bss->chan_idx = freq_to_chan_2g(roam_info->bss_desc->chan_freq);
	ht2g_cur_bss->nss = roam_info->chan_info.nss;
	ht2g_cur_bss->ch_width = roam_info->chan_info.ch_width;
	ht2g_cur_bss->sinr = roam_info->bss_desc->sinr;
	if (ht2g_cur_bss->ch_width == CH_WIDTH_40MHZ) {
		smbw_debug("smbw--sec_ch_offset:%d, @%s, %d",
			ht2g_cur_bss->sec_ch_offset, __func__, __LINE__);
		bw_mgr->ht40_connected = 1;
		bw_mgr->ht20_connected = 0;
		ht2g_cur_bss->weight_adjusted = 0;
		ht2g_cur_bss->weight_adjuste_enable = 0;
		bw_mgr->ll_stats_metric.adaptive_threshold.sample_num = 0;
		bw_mgr->ll_stats_metric.adaptive_threshold.total_rx_mcs = 0;
		bw_mgr->ll_stats_metric.avg_rx_mcs = 0;
	} else if (ht2g_cur_bss->ch_width == CH_WIDTH_20MHZ) {
		bw_mgr->ht20_connected = 1;
		bw_mgr->ht40_connected = 0;
	}
	bw_mgr->ht2g_connected = 1;

	smbw_err("smbw--vdev_id:%d,chan_idx:%d,ch_width:%d,rssi_raw:%d,sec_ch_offset:%d,af_start:%d,af_end:%d,@%s, %d",
		bw_mgr->vdev_id, ht2g_cur_bss->chan_idx, ht2g_cur_bss->ch_width,
		ht2g_cur_bss->rssi_raw, ht2g_cur_bss->sec_ch_offset,
		ht2g_cur_bss->af_chan_start, ht2g_cur_bss->af_chan_end,
		__func__, __LINE__);

	return 0;
}

int enable_2gll_stats_timer(struct hdd_adapter *adapter, int enable,
	unsigned char chan_id,
	unsigned char power_save)
{
	int ret = 0;
	int sample_interval = 0;

	if (unlikely(FEATURE_DISABLE || smart_bw_mgr_g.func_exit || !adapter)) {
		smbw_debug("smbw--feature disabled or NULL pointer!@%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (adapter->device_mode != QDF_STA_MODE ||
		chan_id > 14 || chan_id < 1) {
		smbw_err("smbw--device_mode != QDF_STA_MODE! @%s, %d",
			__func__, __LINE__);
		return -1;
	}

	if (!smart_bw_mgr_g.ht40_connected) {
		smbw_debug("smbw--ht40 not connected!@%s, %d",
			__func__, __LINE__);
		return -1; 
	}

	if (enable) {
		smart_bw_mgr_g.ll_timer_enable = 1;
		sample_interval = g_acs_threshold_cfg.sample_interval;
	} else {
		smbw_debug("smbw--disable 2gll_stats_timer, @%s,%d",
			__func__, __LINE__);
		if (smart_bw_mgr_g.ll_timer_enable == 0) {
			return -1;
		} else {
			smart_bw_mgr_g.ll_timer_enable = 0;
			if (smart_bw_mgr_g.ht40_connected == 1 &&
				power_save == 0) {
				smart_bw_mgr_g.ht2g_cur_bss.chan_idx = 0;
				smart_bw_mgr_g.ht2g_connected = 0;
				smart_bw_mgr_g.ht40_connected = 0;
			}
		}
	}

	ret = wma_cli_set_command(adapter->vdev_id,
		WMI_PDEV_PARAM_STATS_OBSERVATION_PERIOD,
		sample_interval * 1000, PDEV_CMD);

	smbw_err("smbw----set period:%d, @%s, %d", 
		sample_interval, __func__, __LINE__);

	return ret;
}

int handle_sta_disconnect(struct hdd_adapter *adapter, unsigned int disable)
{
	if (unlikely(FEATURE_DISABLE || !adapter)) {
		smbw_debug("smbw--feature disabled or NULL pointer!@%s, %d",
			__func__, __LINE__);
		return -1;
	}

	smbw_err("smbw--adapter->device_mode:%d, disable:%d,  @%s, %d",
	adapter->device_mode, disable, __func__, __LINE__);

	if (disable && smart_bw_mgr_g.ht2g_cur_bss.ht40_support != 0) {
		write_connect_record();
		smbw_err("smbw--disable wlan adapter@%s, %d", __func__, __LINE__);
	}
	if (adapter->device_mode != QDF_STA_MODE) {
		smbw_err("smbw--device_mode != QDF_STA_MODE @%s, %d",
		__func__, __LINE__);
		return -1;
	}


	if (smart_bw_mgr_g.ht40_connected == 1) {
		smart_bw_mgr_g.ht40_connected = 0;
		smart_bw_mgr_g.ll_stats_metric.buffer_desc.head_index = 0;
		smart_bw_mgr_g.ll_stats_metric.buffer_desc.tail_index = 0;
		smart_bw_mgr_g.ll_stats_metric.buffer_desc.cnt = 0;
		smart_bw_mgr_g.ht2g_cur_bss.weight_adjusted = 0;
		smart_bw_mgr_g.ht2g_cur_bss.weight_adjuste_enable = 0;
		smart_bw_mgr_g.ll_stats_metric.adaptive_threshold.sample_num = 0;
		smart_bw_mgr_g.ll_stats_metric.adaptive_threshold.total_rx_mcs = 0;
		smart_bw_mgr_g.ll_stats_metric.avg_rx_mcs = 0;
		smart_bw_mgr_g.ll_stats_metric.adaptive_threshold.total_weight = 0;
		smbw_err("smbw--disconnect ht40@%s, %d", __func__, __LINE__);
	} else if (smart_bw_mgr_g.ht20_connected == 1) {
		smart_bw_mgr_g.ht20_connected = 0;
		smbw_err("smbw--disconnect ht20@%s, %d", __func__, __LINE__);
	}
	smart_bw_mgr_g.ht2g_connected = 0;
	get_config_rom_update();
	if (g_acs_threshold_cfg.feature_enable != 1) {
		smbw_err("smbw--feature force disabled!@%s, %d",
			__func__, __LINE__);
		smart_bw_deinit();
	}

	return 0;
}

static int get_config_rom_update(void)
{
	if (get_smart_bw_rom_update((int*)(&g_acs_threshold_cfg),
			sizeof(smart_bw_rus_cfg_t) / sizeof(int))) {
		return 0;
	} else {
		smbw_debug("smbw--get RUS config failed!!, @%s, %d", __func__, __LINE__);
		return -1;
	}
}

static int smart_bw_setup(void)
{
	smbw_err("smbw--smart_bw_setup start, @%s, %d", __func__, __LINE__);
	smart_bw_mgr_g.chan_stats_queue =
			alloc_ordered_workqueue("CHAN_STATS_QUEUE",
			WQ_POWER_EFFICIENT |
			WQ_HIGHPRI |
			WQ_CPU_INTENSIVE);
	smart_bw_mgr_g.ll_stats_queue =
			alloc_ordered_workqueue("LL_STATS_QUEUE",
			WQ_POWER_EFFICIENT |
			WQ_HIGHPRI |
			WQ_CPU_INTENSIVE);
	if (!smart_bw_mgr_g.chan_stats_queue || !smart_bw_mgr_g.ll_stats_queue) {
		smbw_err("smbw--alloc_ordered_workqueue failed!, @%s, %d", __func__, __LINE__);
		return -1;
	}
	init_buffer_desc(&smart_bw_mgr_g);
	init_manage_bss_list();
	chan_sel_init(&smart_bw_mgr_g);
	INIT_WORK(&smart_bw_mgr_g.chan_stats_work, handle_chan_stats_work);
	INIT_WORK(&smart_bw_mgr_g.ll_stats_work, handle_stats_work);
	init_waitqueue_head(&smart_bw_mgr_g.chan_stats_waitq);
	init_waitqueue_head(&smart_bw_mgr_g.ll_stats_waitq);
	qdf_spinlock_create(&smart_bw_mgr_g.bw_decision.bw_decision_lock);
	qdf_spinlock_create(&smart_bw_mgr_g.ll_stats_metric.ll_metric_lock);
	qdf_spinlock_create(&smart_bw_mgr_g.scan_stats_lock);
	qdf_spinlock_create(&smart_bw_mgr_g.chan_stats_lock);
	qdf_spinlock_create(&smart_bw_mgr_g.chan_weight_lock);
	qdf_spinlock_create(&smart_bw_mgr_g.ll_stats_metric.buffer_desc.stat_buf_lock);
	qdf_spinlock_create(&g_manage_bss_list.bss_list_lock);
	smart_bw_mgr_g.func_enable = 1;
	smbw_err("smbw--smart_bw_setup end, @%s, %d", __func__, __LINE__);

	return 0;
}

int smart_bw_init(struct hdd_context *hdd_ctx)
{
	qdf_mem_zero(&smart_bw_mgr_g, sizeof(struct smart_bw_mgr));

	g_res_stats = &scan_res_stats_g;
	if (!g_res_stats) {
                smbw_err("smbw--NULL pointer! @%s, %d",
                __func__, __LINE__);
                return -1;
	} else {
		qdf_mem_zero(g_res_stats, sizeof(struct scan_res_stats));
	}
	qdf_mem_zero(&g_last_connect_record, sizeof(last_record_t));
	smart_bw_mgr_g.res_stats = g_res_stats;
	smart_bw_mgr_g.sample_interval = g_acs_threshold_cfg.sample_interval;
	smart_bw_mgr_g.func_exit = 0;
	smart_bw_mgr_g.hdd_ctx = hdd_ctx;
	ucfg_mlme_set_channel_bonding_24ghz(hdd_ctx->psoc,
		eHT_CHANNEL_WIDTH_40MHZ);
	smbw_err("smbw--enable channel_bonding_24ghz and snr_monitor@%s, %d",
	__func__, __LINE__);

	return 0;
}

int smart_bw_deinit(void)
{
	smbw_err("smbw--enter+1 @%s, %d", __func__, __LINE__);
	smart_bw_mgr_g.func_exit = 1;
	if (FEATURE_DISABLE) {
		smbw_err("smbw--feature disabled! @%s, %d",
			__func__, __LINE__);
		return -1;
	}
	enable_channel_bonding_24ghz(false);
	wake_up_all(&smart_bw_mgr_g.chan_stats_waitq);
	wake_up_all(&smart_bw_mgr_g.ll_stats_waitq);

	cancel_work_sync(&smart_bw_mgr_g.chan_stats_work);
	flush_work(&smart_bw_mgr_g.chan_stats_work);
	flush_workqueue(smart_bw_mgr_g.chan_stats_queue);
	destroy_workqueue(smart_bw_mgr_g.chan_stats_queue);

	cancel_work_sync(&smart_bw_mgr_g.ll_stats_work);
	flush_work(&smart_bw_mgr_g.ll_stats_work);
	flush_workqueue(smart_bw_mgr_g.ll_stats_queue);
	destroy_workqueue(smart_bw_mgr_g.ll_stats_queue);

	qdf_spinlock_destroy(&smart_bw_mgr_g.bw_decision.bw_decision_lock);
	qdf_spinlock_destroy(&smart_bw_mgr_g.ll_stats_metric.ll_metric_lock);
	qdf_spinlock_destroy(&smart_bw_mgr_g.scan_stats_lock);
	qdf_spinlock_destroy(&smart_bw_mgr_g.chan_stats_lock);
	qdf_spinlock_destroy(&smart_bw_mgr_g.chan_weight_lock);
	qdf_spinlock_destroy(&smart_bw_mgr_g.ll_stats_metric.buffer_desc.stat_buf_lock);
	qdf_spinlock_destroy(&g_manage_bss_list.bss_list_lock);

#if 1
	deinit_manage_bss_list();
#endif
	g_acs_threshold_cfg.feature_enable = 0;
	smart_bw_mgr_g.func_enable = 0;
	smbw_err("smbw--enter+9 @%s, %d", __func__, __LINE__);

	return 0;
}

void smbw_deinit_low_layer(void)
{
	if (g_acs_threshold_cfg.feature_enable) {
		smbw_err("smbw--smbw func not exit yet!! @%s, %d",
			__func__, __LINE__);
		smart_bw_deinit();
	} else {
		smbw_err("smbw--smbw func exit @%s, %d",
			__func__, __LINE__);
	}
}

void check_rus_status(void)
{
	if (smart_bw_mgr_g.rus_ready && smart_bw_mgr_g.func_enable)
		return;

	if (get_config_rom_update()) {
		return;
	} else {
		smart_bw_mgr_g.rus_ready = 1;
		smbw_err("smbw--feature_enable:%d, debug_level:%d, ll_com_score_thre:%d,@%s, %d",
		g_acs_threshold_cfg.feature_enable,
		g_acs_threshold_cfg.debug_level,
		g_acs_threshold_cfg.ll_com_score_thre,
		__func__, __LINE__);

		smbw_err("smbw--acs_weight_thre:%d, mov_avg_beta:%d, thre_tune_dist:%d,@%s, %d",
		g_acs_threshold_cfg.acs_weight_thre,
		g_acs_threshold_cfg.mov_avg_beta,
		g_acs_threshold_cfg.thre_tune_dist,
		__func__, __LINE__);

		smbw_err("smbw--sample_interval:%d, good_mcs:%d, bad_mcs:%d,@%s, %d",
		g_acs_threshold_cfg.sample_interval,
		g_acs_threshold_cfg.good_mcs,
		g_acs_threshold_cfg.bad_mcs,
		__func__, __LINE__);
		if (g_acs_threshold_cfg.feature_enable)
			smart_bw_setup();
	}
}
