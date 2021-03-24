#ifndef __WLAN_2G_HT40_H__
#define __WLAN_2G_HT40_H__
#include <linux/wireless.h>
#include <linux/netdevice.h>
#include <stddef.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/timer.h>
#include <net/cfg80211.h>
#include <linux/string.h>
#include "../../../qca-wifi-host-cmn/qdf/inc/qdf_list.h"
#include "../../core/mac/inc/sir_api.h"
#include "../../../fw-api/fw/wmi_unified.h"
#include "../../../qca-wifi-host-cmn/umac/scan/core/src/wlan_scan_main.h"
#include "../../components/cmn_services/policy_mgr/inc/wlan_policy_mgr_api.h"
#include "../../../qca-wifi-host-cmn/qdf/inc/qdf_atomic.h"
#include "../../../qca-wifi-host-cmn/umac/cmn_services/obj_mgr/inc/wlan_objmgr_pdev_obj.h"
#include "../../core/mac/inc/ani_global.h"
#include "../../core/mac/src/include/dot11f.h"
#include "../../core/mac/inc/sir_mac_prot_def.h"
#include "../../components/mlme/core/inc/wlan_mlme_main.h"
#include "../../../qca-wifi-host-cmn/umac/scan/dispatcher/inc/wlan_scan_ucfg_api.h"
#include "../../../qca-wifi-host-cmn/qdf/inc/qdf_lock.h"
#include "../../../qca-wifi-host-cmn/umac/scan/dispatcher/inc/wlan_scan_public_structs.h"
#include "../../hdd/inc/wlan_hdd_main.h"
#include "../../../qca-wifi-host-cmn/umac/cmn_services/inc/wlan_cmn.h"
#include "../../../qca-wifi-host-cmn/umac/scan/dispatcher/inc/wlan_scan_utils_api.h"
#include "../../../qca-wifi-host-cmn/qdf/inc/qdf_util.h"
#include "../../components/mlme/dispatcher/inc/wlan_mlme_ucfg_api.h"
#include "../../core/wma/inc/wma_api.h"
#include "../../../qca-wifi-host-cmn/umac/scan/core/src/wlan_scan_cache_db_i.h"
#include "../../core/sme/inc/csr_api.h"
#include "../../../qca-wifi-host-cmn/qdf/inc/qdf_mem.h"
#include "../../core/mac/src/include/parser_api.h"
#include "../../core/mac/src/pe/include/lim_session.h"

#ifndef ETH_ALEN
#define ETH_ALEN			6
#endif

#ifndef CH_WIDTH_20MHZ
#define CH_WIDTH_20MHZ		0
#define CH_WIDTH_40MHZ		1
#endif

#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#ifndef BAND_2G
#define BAND_2G 1
#define BAND_5G 2
#endif

#define LL_COM_SCORE_THREAD 440
#define ACS_WEIGHT_THRE		34100
#define MOV_AVG_BETA        8

#define ACS_WEIGHT_MAX		(26664)
#define SAMPLE_INTERVAL		2
#define ACS_WEIGHT			16448250
#define BG_SCORE_THRES		8
#define INTOLERANT_NUM_THRES 2
#define END_CHAN_FREQ_2G	2472
#define MAX_ESTIMATED_AIR_TIME_FRACTION 255
#define MAX_AP_LOAD			255
#define MAX_SCAN_RES_NUM	120

#define MAX_2G_CHAN_NUM		14
#define LEGACY_BSS_WEIGHT	3
#define AFFECTED_FREQ_DIST	5
#define SMBW_MAX_CHANNEL_UTILIZATION 100
#define SPECT_24GHZ_CH_COUNT	(11)    /* USA regulatory domain */
#define ACS_NORMALISE_1000	(1000/9)  /* Case of spec20 with channel diff = 0 */

#define ACS_MIN_RSSI         (-100)
#define ACS_MAX_RSSI         (0)
#define ACS_MIN_COUNT        (0)
#define ACS_MAX_COUNT        (60)

#define ACS_MIN_NF           (-120)
#define ACS_MAX_NF           (-60)
#define ACS_MIN_CHNFREE      (0)
#define ACS_MAX_CHNFREE      (1)
#define ACS_MIN_TXPWR        (0)
#define ACS_MAX_TXPWR        (63)

#define AFFCTED_FREQ_RANGE	25
#define CHANNEL_GAP_2G		5
#define HT40_CHAN_PAIR_OFFSET 4
#define HT_CAP_INFO_40MHZ_INTOLERANT	((unsigned short) BIT(14))
#define LOW_RSSI_THRES		(-95)

/* In 2.4GHZ, Effect of Primary  Channel RSSI on First Overlapping Channel */
#define ACS_24GHZ_FIRST_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY      (-10)
/* In 2.4GHZ, Effect of Primary  Channel RSSI on Second Overlapping Channel */
#define ACS_24GHZ_SEC_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY        (-20)
/* In 2.4GHZ, Effect of Primary  Channel RSSI on Third Overlapping Channel */
#define ACS_24GHZ_THIRD_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY      (-30)
/* In 2.4GHZ, Effect of Primary  Channel RSSI on Fourth Overlapping Channel */
#define ACS_24GHZ_FOURTH_OVERLAP_CHAN_RSSI_EFFECT_PRIMARY     (-40)

#define IS_RSSI_VALID(extRssi, rssi) \
	( \
	((extRssi < rssi) ? true : false) \
	)

typedef struct {
	uint16_t chNum;         /* Channel Number */
	uint16_t channelWidth;  /* Channel Width */
	uint16_t bssCount;      /* bss found in scanresult for this channel */
	int32_t rssiAgr;        /* Max value of rssi among all BSS(es) from scanresult for this channel */
	uint32_t weight;        /* Weightage of this channel */
	uint32_t weight_copy;   /* copy of the orignal weight */
	bool valid;             /* Is this a valid center frequency for regulatory domain */
} tAcsSpectChInfo;              /* tDfsSpectChInfo; */

/* HT common IE info */
struct ht_common_ie {
	uint16_t hc_cap;	/* HT capabilities */
	uint8_t ampdu_param;
	uint8_t mcsset[16];
	uint16_t extcap;
	uint32_t txbf_cap;
	uint8_t antenna;
} qdf_packed;

/**
 * Structure holding all the information required to make a
 * decision for the best operating channel based on dfs formula
 */
typedef struct {
	tAcsSpectChInfo pSpectCh[14];
	uint8_t numSpectChans;  /* Total num of channels in the spectrum */
} tAcsChSelSpectInfo;

typedef enum {
	CHANNEL_1 = 1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7,
	CHANNEL_8,
	CHANNEL_9,
	CHANNEL_10,
	CHANNEL_11,
	CHANNEL_12,
	CHANNEL_13,
	CHANNEL_14
} Channel_list;

typedef enum {
	scan_band_ratio,
	connect_band_ratio,
	smbw_max,
} wcn_key_log_id;

typedef struct {
	int    feature_enable;
	int    debug_level;
	int    ll_com_score_thre;
	int    acs_weight_thre;
	int    mov_avg_beta;
	int    thre_tune_dist;
	int    sample_interval;
	int    good_mcs;
	int    bad_mcs;
} smart_bw_rus_cfg_t;

typedef struct {
	ktime_t         time_stamp;
	unsigned char	ht40_enable;
	unsigned char	updated;
	unsigned char	ht40_intolerant;
	unsigned char	ht40_bg_enable;
	unsigned char	exist_ht40_conflict;
	unsigned char	ht40_chan_busy;
	unsigned int	ht40p_weight;
	unsigned int	ht40m_weight;
} last_record_t;

typedef struct {
	unsigned char   pri_rssi_weight;
	unsigned char   pri_count_weight;
	unsigned char   pri_noise_floor_weight;
	unsigned char   sec_rssi_weight;
	unsigned char   sec_count_weight;
	unsigned char   sec_noise_floor_weight;
	unsigned char   channel_free_weight;
} cur_weight_info_t;

typedef enum {
	PRI_RSSSI,
	PRI_BSS_CNT,
	PRI_NOISE_FLOOR,
	SEC_RSSSI,
	SEC_BSS_CNT,
	SEC_NOISE_FLOOR,
	CHAN_FREE,
} weight_id;

struct ht2g_bss_info {
	unsigned char		bssid[ETH_ALEN];
	unsigned char		nss;
	unsigned char		ht40_support;
	unsigned char		chan_idx;
	unsigned char		sec_chan_idx;
	int 		        sec_ch_offset;
	int			rssi_raw;
	int 			sinr;
	unsigned long 		tsf;
	unsigned long 		parent_tsf;
	unsigned short 		ht_caps;
	unsigned char		af_chan_start;
	unsigned char		af_chan_end;
	unsigned char       	weight_adjusted;
	unsigned char       	weight_adjuste_enable;
	unsigned int        	avg_weight;
	cur_weight_info_t   	cur_weight_info;
	char			noise;
	unsigned char       	need_post_lq_log;
	unsigned char       	need_post_chan_log;
	enum phy_ch_width 	ch_width;
	enum wlan_phymode	phy_mode;
};

typedef struct {
	qdf_list_node_t     node;
	unsigned char		bssid[ETH_ALEN];
	unsigned char       nss;
	unsigned char		chan_idx;
	unsigned char 		sec_ch_offset;
	unsigned char       avg_rx_mcs;
	unsigned int        avg_weight;
	int 				rssi_raw;
	int                 snr;
} ht40_bss_node_t;

typedef struct {
	qdf_list_t      white_bss_list_head;
	qdf_list_t      black_bss_list_head;
	qdf_spinlock_t 	bss_list_lock;
} manage_bss_list_t;

typedef struct {
	unsigned char channel_id;
	/*cnt of BSS using this as their ctl channel */
	unsigned char nCtrl;
	/* cnt of 40MHZBSS using this as their ext20 channel */
	unsigned char nExt20;
} scan_chan_bssinfo_t;

struct scan_res_stats {
	unsigned short		scan_id;
	unsigned long		tsf;
	unsigned char		ht40_intolerant[MAX_2G_CHAN_NUM + 1];
	unsigned char		ht40_bss_num;
	unsigned char		bg_bss_num[MAX_2G_CHAN_NUM + 1];
	unsigned char		ht40_not_align;
	scan_chan_bssinfo_t ht40_chan_bssinfo[MAX_2G_CHAN_NUM + 1];
};

typedef struct {
	unsigned char bssid[ETH_ALEN];
	unsigned char chan_id;
	int rssi_raw;
	int chan_offset;
} ht40_bss_t;

typedef struct {
	ktime_t			time_stamp;
	qdf_spinlock_t		scan_res_lock;
	unsigned int		bss_num;
	unsigned int		ht40_bss_num;
	ht40_bss_t              ht40_bss[20];
} smbw_scan_results_t;

struct ht40_bw_decision {
	unsigned char		ht40_scan_enable;
	unsigned char		ht40_ll_enable;
	unsigned char		ht40_intolerant;
	unsigned char		ht40_bg_enable;
	unsigned char		exist_ht40_conflict;
	unsigned char		ht40_chan_enable;
	unsigned char		ht40_enable;
	qdf_spinlock_t 		bw_decision_lock;
	ktime_t			dec_time_stamp;
};

typedef struct {
	ktime_t		time_stamp;
	unsigned int	usage_ratio;
} media_usage_ratio_t;

typedef struct {
	unsigned int	chan_id;
	unsigned long	radio_awake_time;
	unsigned long	cca_busy_time;
} channel_stat_t;

typedef struct {
	ktime_t		time_stamp;
	unsigned int	stat_num;
	channel_stat_t	chan_stat[32];
} cca_busy_state_t;

typedef struct {
	ktime_t		time_stamp;
	unsigned int	cca_busy_ratio[MAX_2G_CHAN_NUM + 1];
} cca_busy_ratio_t;

typedef struct {
	ktime_t		time_stamp;
	unsigned int	cca_busy_score;
} cca_busy_score_t;

typedef struct {
	ktime_t		time_stamp;
	unsigned int	fail_ratio;
} tx_fail_ratio_t;

typedef struct {
	ktime_t		time_stamp;
	unsigned int	fail_ratio;
} rx_fail_ratio_t;

typedef struct {
	struct wifi_rate_info	rate;
	unsigned int		mpdu_lost_ratio;
	unsigned int		mpdu_retries_ratio;
} rate_stat_ratio_t;

typedef struct {
	unsigned char	radio_sampled;
	unsigned char	iface_sampled;
	unsigned char	sta_stats_sampled;
} sample_state_t;

typedef struct {
	unsigned char		radio_id_2g;
	unsigned char		vdev_id;
	unsigned char		sample_ch_num;
	sample_state_t		sample_state;
	unsigned int		rsp_id;
	unsigned long		ll_report_interval;
	cca_busy_state_t	cca_busy_state;
	unsigned long		parent_data_num;
	ktime_t			cal_base_time;
	ktime_t			cal_parent_time;
	ktime_t			cur_time;
	unsigned int		tsf_delta;
} ll_stats_t;

typedef struct {
	int         rssi;
	int8_t      snr;
	uint8_t     tx_nss;
	uint8_t     rx_nss;
	uint32_t    max_pwr;
	uint32_t    tx_rate;
	uint32_t    rx_rate;
	uint32_t    tx_mcs_index;
	uint32_t    rx_mcs_index;
	uint8_t     tx_dcm;
	uint8_t     rx_dcm;
	uint8_t     tx_gi;
	uint8_t     rx_gi;
} sta_2g_rate_info_t;

typedef struct {
	media_usage_ratio_t	media_usage_ratio;
	cca_busy_ratio_t	cca_busy_ratio;
	tx_fail_ratio_t		tx_fail_ratio;
	rx_fail_ratio_t		rx_fail_ratio;
	cca_busy_score_t	cur_cca_busy_score;
	unsigned long		data_flow;
	unsigned char		stat_num_chan[MAX_2G_CHAN_NUM + 1];
	unsigned char		cur_ht40_stat_num;
	sta_2g_rate_info_t      sta_rate_info;
} stats_metric_t;

typedef struct {
	unsigned int		head_index;
	unsigned int		tail_index;
	unsigned int		cnt;
	qdf_spinlock_t		stat_buf_lock;
}  buffer_desc_t;

typedef struct {
	unsigned int    total_rx_mcs;
	unsigned long   total_weight;
	unsigned int    sample_num;
} adaptive_threshold_t;

typedef struct {
	unsigned int		avg_tx_fail_score;
	unsigned int		avg_rx_fail_score;
	unsigned int		avg_cur_cca_busy_score;
	unsigned int            avg_rx_mcs;
	//unsigned int            avg_weight;
	unsigned int            adaptive_avg_weight;
	unsigned long		avg_data_flow_score;
	unsigned char           tx_nss;
	unsigned char           rx_nss;
	stats_metric_t		stats_metric[6];
	buffer_desc_t		buffer_desc;
	adaptive_threshold_t    adaptive_threshold;
	enum tx_rate_info       tx_rx_rate_flags;
	int                     avg_rssi;
	qdf_spinlock_t          ll_metric_lock;
} ll_stats_metric_t;

struct smart_bw_mgr {
	struct work_struct		chan_stats_work;
	struct work_struct		ll_stats_work;
	tAcsChSelSpectInfo		SpectInfoParams;
	unsigned int			cur_weight;
	ktime_t				cur_weight_timestamp;
	unsigned int			parent_cur_weight;
	ktime_t				parent_weight_timestamp;
	struct lim_channel_status	chan_stats[14];
	unsigned char			chan_free_weight[14];
	unsigned int			chan_weight[14];
	ktime_t				chan_weight_timestamp;
	unsigned char			chan_stats_cnt;
	unsigned char			handle_chan_stats_going;
	qdf_spinlock_t			chan_weight_lock;
	qdf_spinlock_t			chan_stats_lock;
	unsigned int			last_chan_stat_freq;
	unsigned char			sample_interval;
	struct scan_res_stats		*res_stats;
	smbw_scan_results_t		scan_results;
	unsigned int			chan_stats_wake;
	unsigned int			ll_stats_wake;
	ll_stats_t			ll_stats;
	ll_stats_metric_t		ll_stats_metric;
	struct ht40_bw_decision 	bw_decision;
	struct ht2g_bss_info 		ht2g_cur_bss;
	unsigned char			ht40_connected;
	unsigned char			ht20_connected;
	unsigned char			ht2g_connected;
	unsigned char			radio_id_2g;
	unsigned char			mcc_mode;
	unsigned char			vdev_id;
	unsigned char			ll_timer_enable;
	unsigned char			func_exit;
	unsigned char			rus_ready;
	unsigned char			func_enable;
	unsigned char			connect_record_expire;
	unsigned char			hdd_stop_adapter;
	wait_queue_head_t		chan_stats_waitq;
	wait_queue_head_t		ll_stats_waitq;
	qdf_spinlock_t			scan_stats_lock;
	struct workqueue_struct 	*chan_stats_queue;
	struct workqueue_struct 	*ll_stats_queue;
	struct wlan_objmgr_pdev		*pdev;
	struct hdd_context		*hdd_ctx;
	struct hdd_adapter		*adapter;
	struct mac_context		*mac;
};

int smart_bw_deinit(void);
int smbw_ll_process_radio_stats(tSirLLStatsResults *link_stats,
	struct wifi_radio_stats *rs_results, 
	wmi_channel_stats *channel_stats);
int check_ht40_bw_enable(struct mac_context *mac, tDot11fIEHTCaps *pDot11f,
	struct pe_session *pe_session);
int get_ht2g_connected_bss(struct hdd_adapter *adapter,
	struct csr_roam_info *roam_info);
int enable_2gll_stats_timer(struct hdd_adapter *adapter, int enable,
	unsigned char chan_id, unsigned char power_save);
int smbw_ll_process_iface_stats(wmi_iface_link_stats *link_stats,
	wmi_wmm_ac_stats *iface_ac_stats,
	unsigned int vdev_id);
int handle_scan_chan_stats(struct wlan_objmgr_pdev *pdev,
	wmi_chan_info_event_fixed_param *event);
int handle_sta_disconnect(struct hdd_adapter *adapter, unsigned int disable);
void smbw_deinit_low_layer(void);
int get_sta_stats(struct hdd_adapter *adapter);
int smart_bw_init(struct hdd_context *hdd_ctx);
void check_rus_status(void);
void set_snr_monitor_enable(struct hdd_context *hdd_ctx, int enable);
#endif
