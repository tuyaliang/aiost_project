/* common shared memory */
#define SHMEM_TOP_ADDR_1GB		0x46900000
#define SHMEM_TOP_ADDR_2GB		0x4FF00000
#define SHMEM_TOP_ADDR_VBD		0x45400000
#define SHMEM_TOP_ADDR_VBD_ACT		0x45F00000
#define SHMEM_GR_IPCU_SIZE	(0x300)
#define SHMEM_TRAMPOLINE_SIZE                0x100
#define SHMEM_TOP_ADDR_SIZE     (0x00100000-SHMEM_GR_IPCU_SIZE-SHMEM_TRAMPOLINE_SIZE)
#define SHMEM_RTCP_DATA_SIZE	(0x1000)

#define SHMEM_IPCU_BUFFER_ADDR_OFFSET			0x04
#define SHMEM_IPCU_BUFFER_SIZE_OFFSET			0x08
#define SHMEM_IPCU_SYNC_ADDR_OFFSET			0x0C
#define SHMEM_IPCU_SYNC_SIZE_OFFSET			0x10
#define SHMEM_TS_READ_POINTER_ADDR_OFFSET		0x14
#define SHMEM_TS_READ_POINTER_SIZE_OFFSET		0x18
#define SHMEM_TS_WRITE_POINTER_ADDR_OFFSET		0x1C
#define SHMEM_TS_WRITE_POINTER_SIZE_OFFSET		0x20
#define SHMEM_MOVIE_RECORD0_DFS_ADDR_OFFSET		0x24
#define SHMEM_MOVIE_RECORD0_DFS_SIZE_OFFSET		0x28
#define SHMEM_TERMINAL_IO_ADDR_OFFSET			0x2C
#define SHMEM_TERMINAL_IO_SIZE_OFFSET			0x30
#define SHMEM_STRING_ADDR_OFFSET			0x34
#define SHMEM_STRING_SIZE_OFFSET			0x38
#define SHMEM_RIBERY_STREAM_ADDR_OFFSET			0x3C
#define SHMEM_RIBERY_STREAM_SIZE_OFFSET			0x40
#define SHMEM_AUDIO_STREAM_ADDR_OFFSET			0x44
#define SHMEM_AUDIO_STREAM_SIZE_OFFSET			0x48
#define SHMEM_RAW_STREAM_ADDR_OFFSET			0x4C
#define SHMEM_RAW_STREAM_SIZE_OFFSET			0x50
#define SHMEM_YUV_STREAM_ADDR_OFFSET			0x54
#define SHMEM_YUV_STREAM_SIZE_OFFSET			0x58
#define SHMEM_HEVC_STREAM_ADDR_OFFSET			0x5C
#define SHMEM_HEVC_STREAM_SIZE_OFFSET			0x60
#define SHMEM_MJPEG_STREAM_ADDR_OFFSET			0x64
#define SHMEM_MJPEG_STREAM_SIZE_OFFSET			0x68
#define SHMEM_CAP_INFO_ADDR_OFFSET			0x6C
#define SHMEM_CAP_INFO_SIZE_OFFSET			0x70
#define SHMEM_OSD_INFO_ADDR_OFFSET			0x74
#define SHMEM_OSD_INFO_SIZE_OFFSET			0x78
#define SHMEM_AUDIO_OUT_ADDR_OFFSET			0x7C
#define SHMEM_AUDIO_OUT_SIZE_OFFSET			0x80
#define SHMEM_METADATA_ADDR_OFFSET			0x84
#define SHMEM_METADATA_SIZE_OFFSET			0x88
#define SHMEM_RTCP_DATA_OFFSET					(0xFE000)
/* macro */
/* 1GB */
#define GET_IPCU_BUFFER_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_IPCU_BUFFER_ADDR_OFFSET)
#define GET_IPCU_BUFFER_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_IPCU_BUFFER_SIZE_OFFSET)
#define GET_IPCU_SYNC_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_IPCU_SYNC_ADDR_OFFSET)
#define GET_IPCU_SYNC_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_IPCU_SYNC_SIZE_OFFSET)
#define GET_TS_READ_POINTER_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TS_READ_POINTER_ADDR_OFFSET)
#define GET_TS_READ_POINTER_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TS_READ_POINTER_SIZE_OFFSET)
#define GET_TS_WRITE_POINTER_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TS_WRITE_POINTER_ADDR_OFFSET)
#define GET_TS_WRITE_POINTER_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TS_WRITE_POINTER_SIZE_OFFSET)
#define GET_MOVIE_RECORD0_DFS_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_MOVIE_RECORD0_DFS_ADDR_OFFSET)
#define GET_MOVIE_RECORD0_DFS_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_MOVIE_RECORD0_DFS_SIZE_OFFSET)
#define GET_TERMINAL_IO_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TERMINAL_IO_ADDR_OFFSET)
#define GET_TERMINAL_IO_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TERMINAL_IO_SIZE_OFFSET)
#define GET_STRING_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_STRING_ADDR_OFFSET)
#define GET_STRING_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_STRING_SIZE_OFFSET)
#define GET_RIBERY_STREAM_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_RIBERY_STREAM_ADDR_OFFSET)
#define GET_RIBERY_STREAM_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_RIBERY_STREAM_SIZE_OFFSET)
#define GET_AUDIO_STREAM_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_AUDIO_STREAM_ADDR_OFFSET)
#define GET_AUDIO_STREAM_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_AUDIO_STREAM_SIZE_OFFSET)
#define GET_RAW_STREAM_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_RAW_STREAM_ADDR_OFFSET)
#define GET_RAW_STREAM_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_RAW_STREAM_SIZE_OFFSET)
#define GET_YUV_STREAM_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_YUV_STREAM_ADDR_OFFSET)
#define GET_YUV_STREAM_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_YUV_STREAM_SIZE_OFFSET)
#define GET_HEVC_STREAM_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_HEVC_STREAM_ADDR_OFFSET)
#define GET_HEVC_STREAM_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_HEVC_STREAM_SIZE_OFFSET)
#define GET_MJPEG_STREAM_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_MJPEG_STREAM_ADDR_OFFSET)
#define GET_MJPEG_STREAM_SIZE_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_MJPEG_STREAM_SIZE_OFFSET)
#define GET_CAP_INFO_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_CAP_INFO_ADDR_OFFSET)
#define GET_CAP_INFO_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_CAP_INFO_SIZE_OFFSET)
#define GET_OSD_INFO_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_OSD_INFO_ADDR_OFFSET)
#define GET_OSD_INFO_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_OSD_INFO_SIZE_OFFSET)
#define GET_AUDIO_OUT_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_AUDIO_OUT_ADDR_OFFSET)
#define GET_AUDIO_OUT_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_AUDIO_OUT_SIZE_OFFSET)
#define GET_METADATA_ADDR_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_METADATA_ADDR_OFFSET)
#define GET_METADATA_SIZE_1GB		(SHMEM_TOP_ADDR_1GB + SHMEM_METADATA_SIZE_OFFSET)
/* RTCP sending buffer address */
#define GET_RTCP_DATA_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_RTCP_DATA_OFFSET)
/* Suspend/resume address */
#define SHMEM_TRAMPOLINE_OFFSET_1G (SHMEM_TOP_ADDR_1GB + SHMEM_TOP_ADDR_SIZE + SHMEM_GR_IPCU_SIZE)
/* GR IPCU command buffer address */
#define GET_DISP_SETTING_ADDR_1GB	(SHMEM_TOP_ADDR_1GB + SHMEM_TOP_ADDR_SIZE)


/* 2GB */
#define GET_IPCU_BUFFER_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_IPCU_BUFFER_ADDR_OFFSET)
#define GET_IPCU_BUFFER_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_IPCU_BUFFER_SIZE_OFFSET)
#define GET_IPCU_SYNC_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_IPCU_SYNC_ADDR_OFFSET)
#define GET_IPCU_SYNC_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_IPCU_SYNC_SIZE_OFFSET)
#define GET_TS_READ_POINTER_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TS_READ_POINTER_ADDR_OFFSET)
#define GET_TS_READ_POINTER_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TS_READ_POINTER_SIZE_OFFSET)
#define GET_TS_WRITE_POINTER_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TS_WRITE_POINTER_ADDR_OFFSET)
#define GET_TS_WRITE_POINTER_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TS_WRITE_POINTER_SIZE_OFFSET)
#define GET_MOVIE_RECORD0_DFS_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_MOVIE_RECORD0_DFS_ADDR_OFFSET)
#define GET_MOVIE_RECORD0_DFS_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_MOVIE_RECORD0_DFS_SIZE_OFFSET)
#define GET_TERMINAL_IO_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TERMINAL_IO_ADDR_OFFSET)
#define GET_TERMINAL_IO_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TERMINAL_IO_SIZE_OFFSET)
#define GET_STRING_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_STRING_ADDR_OFFSET)
#define GET_STRING_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_STRING_SIZE_OFFSET)
#define GET_RIBERY_STREAM_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_RIBERY_STREAM_ADDR_OFFSET)
#define GET_RIBERY_STREAM_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_RIBERY_STREAM_SIZE_OFFSET)
#define GET_AUDIO_STREAM_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_AUDIO_STREAM_ADDR_OFFSET)
#define GET_AUDIO_STREAM_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_AUDIO_STREAM_SIZE_OFFSET)
#define GET_RAW_STREAM_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_RAW_STREAM_ADDR_OFFSET)
#define GET_RAW_STREAM_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_RAW_STREAM_SIZE_OFFSET)
#define GET_YUV_STREAM_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_YUV_STREAM_ADDR_OFFSET)
#define GET_YUV_STREAM_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_YUV_STREAM_SIZE_OFFSET)
#define GET_HEVC_STREAM_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_HEVC_STREAM_ADDR_OFFSET)
#define GET_HEVC_STREAM_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_HEVC_STREAM_SIZE_OFFSET)
#define GET_MJPEG_STREAM_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_MJPEG_STREAM_ADDR_OFFSET)
#define GET_MJPEG_STREAM_SIZE_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_MJPEG_STREAM_SIZE_OFFSET)
#define GET_CAP_INFO_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_CAP_INFO_ADDR_OFFSET)
#define GET_CAP_INFO_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_CAP_INFO_SIZE_OFFSET)
#define GET_OSD_INFO_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_OSD_INFO_ADDR_OFFSET)
#define GET_OSD_INFO_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_OSD_INFO_SIZE_OFFSET)
#define GET_AUDIO_OUT_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_AUDIO_OUT_ADDR_OFFSET)
#define GET_AUDIO_OUT_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_AUDIO_OUT_SIZE_OFFSET)
#define GET_METADATA_ADDR_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_METADATA_ADDR_OFFSET)
#define GET_METADATA_SIZE_2GB		(SHMEM_TOP_ADDR_2GB + SHMEM_METADATA_SIZE_OFFSET)
/* RTCP sending buffer address */
#define GET_RTCP_DATA_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_RTCP_DATA_OFFSET)
/* Suspend/resume address */
#define SHMEM_TRAMPOLINE_OFFSET_2G (SHMEM_TOP_ADDR_2GB+SHMEM_TOP_ADDR_SIZE+ SHMEM_GR_IPCU_SIZE)
/* GR IPCU command buffer address */
#define GET_DISP_SETTING_ADDR_2GB	(SHMEM_TOP_ADDR_2GB + SHMEM_TOP_ADDR_SIZE)


/* VBD */
#define GET_IPCU_BUFFER_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_IPCU_BUFFER_ADDR_OFFSET)
#define GET_IPCU_BUFFER_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_IPCU_BUFFER_SIZE_OFFSET)
#define GET_IPCU_SYNC_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_IPCU_SYNC_ADDR_OFFSET)
#define GET_IPCU_SYNC_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_IPCU_SYNC_SIZE_OFFSET)
#define GET_TS_READ_POINTER_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TS_READ_POINTER_ADDR_OFFSET)
#define GET_TS_READ_POINTER_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TS_READ_POINTER_SIZE_OFFSET)
#define GET_TS_WRITE_POINTER_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TS_WRITE_POINTER_ADDR_OFFSET)
#define GET_TS_WRITE_POINTER_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TS_WRITE_POINTER_SIZE_OFFSET)
#define GET_MOVIE_RECORD0_DFS_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_MOVIE_RECORD0_DFS_ADDR_OFFSET)
#define GET_MOVIE_RECORD0_DFS_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_MOVIE_RECORD0_DFS_SIZE_OFFSET)
#define GET_TERMINAL_IO_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TERMINAL_IO_ADDR_OFFSET)
#define GET_TERMINAL_IO_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TERMINAL_IO_SIZE_OFFSET)
#define GET_STRING_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_STRING_ADDR_OFFSET)
#define GET_STRING_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_STRING_SIZE_OFFSET)
#define GET_RIBERY_STREAM_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_RIBERY_STREAM_ADDR_OFFSET)
#define GET_RIBERY_STREAM_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_RIBERY_STREAM_SIZE_OFFSET)
#define GET_AUDIO_STREAM_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_AUDIO_STREAM_ADDR_OFFSET)
#define GET_AUDIO_STREAM_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_AUDIO_STREAM_SIZE_OFFSET)
#define GET_RAW_STREAM_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_RAW_STREAM_ADDR_OFFSET)
#define GET_RAW_STREAM_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_RAW_STREAM_SIZE_OFFSET)
#define GET_YUV_STREAM_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_YUV_STREAM_ADDR_OFFSET)
#define GET_YUV_STREAM_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_YUV_STREAM_SIZE_OFFSET)
#define GET_HEVC_STREAM_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_HEVC_STREAM_ADDR_OFFSET)
#define GET_HEVC_STREAM_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_HEVC_STREAM_SIZE_OFFSET)
#define GET_MJPEG_STREAM_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_MJPEG_STREAM_ADDR_OFFSET)
#define GET_MJPEG_STREAM_SIZE_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_MJPEG_STREAM_SIZE_OFFSET)
#define GET_CAP_INFO_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_CAP_INFO_ADDR_OFFSET)
#define GET_CAP_INFO_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_CAP_INFO_SIZE_OFFSET)
#define GET_OSD_INFO_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_OSD_INFO_ADDR_OFFSET)
#define GET_OSD_INFO_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_OSD_INFO_SIZE_OFFSET)
#define GET_AUDIO_OUT_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_AUDIO_OUT_ADDR_OFFSET)
#define GET_AUDIO_OUT_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_AUDIO_OUT_SIZE_OFFSET)
#define GET_METADATA_ADDR_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_METADATA_ADDR_OFFSET)
#define GET_METADATA_SIZE_VBD		(SHMEM_TOP_ADDR_VBD + SHMEM_METADATA_SIZE_OFFSET)
/* RTCP sending buffer address */
#define GET_RTCP_DATA_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_RTCP_DATA_OFFSET)
/* Suspend/resume address */
#define SHMEM_TRAMPOLINE_OFFSET_VBD (SHMEM_TOP_ADDR_VBD + SHMEM_TOP_ADDR_SIZE + SHMEM_GR_IPCU_SIZE)
/* GR IPCU command buffer address */
#define GET_DISP_SETTING_ADDR_VBD	(SHMEM_TOP_ADDR_VBD + SHMEM_TOP_ADDR_SIZE)

/* VBD ACT */
#define GET_IPCU_BUFFER_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_IPCU_BUFFER_ADDR_OFFSET)
#define GET_IPCU_BUFFER_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_IPCU_BUFFER_SIZE_OFFSET)
#define GET_IPCU_SYNC_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_IPCU_SYNC_ADDR_OFFSET)
#define GET_IPCU_SYNC_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_IPCU_SYNC_SIZE_OFFSET)
#define GET_TS_READ_POINTER_ADDR_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TS_READ_POINTER_ADDR_OFFSET)
#define GET_TS_READ_POINTER_SIZE_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TS_READ_POINTER_SIZE_OFFSET)
#define GET_TS_WRITE_POINTER_ADDR_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TS_WRITE_POINTER_ADDR_OFFSET)
#define GET_TS_WRITE_POINTER_SIZE_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TS_WRITE_POINTER_SIZE_OFFSET)
#define GET_MOVIE_RECORD0_DFS_ADDR_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_MOVIE_RECORD0_DFS_ADDR_OFFSET)
#define GET_MOVIE_RECORD0_DFS_SIZE_VBD_ACT	(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_MOVIE_RECORD0_DFS_SIZE_OFFSET)
#define GET_TERMINAL_IO_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TERMINAL_IO_ADDR_OFFSET)
#define GET_TERMINAL_IO_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TERMINAL_IO_SIZE_OFFSET)
#define GET_STRING_ADDR_VBD_ACT			(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_STRING_ADDR_OFFSET)
#define GET_STRING_SIZE_VBD_ACT			(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_STRING_SIZE_OFFSET)
#define GET_RIBERY_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_RIBERY_STREAM_ADDR_OFFSET)
#define GET_RIBERY_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_RIBERY_STREAM_SIZE_OFFSET)
#define GET_AUDIO_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_AUDIO_STREAM_ADDR_OFFSET)
#define GET_AUDIO_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_AUDIO_STREAM_SIZE_OFFSET)
#define GET_RAW_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_RAW_STREAM_ADDR_OFFSET)
#define GET_RAW_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_RAW_STREAM_SIZE_OFFSET)
#define GET_YUV_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_YUV_STREAM_ADDR_OFFSET)
#define GET_YUV_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_YUV_STREAM_SIZE_OFFSET)
#define GET_HEVC_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_HEVC_STREAM_ADDR_OFFSET)
#define GET_HEVC_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_HEVC_STREAM_SIZE_OFFSET)
#define GET_MJPEG_STREAM_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_MJPEG_STREAM_ADDR_OFFSET)
#define GET_MJPEG_STREAM_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_MJPEG_STREAM_SIZE_OFFSET)
#define GET_CAP_INFO_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_CAP_INFO_ADDR_OFFSET)
#define GET_CAP_INFO_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_CAP_INFO_SIZE_OFFSET)
#define GET_OSD_INFO_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_OSD_INFO_ADDR_OFFSET)
#define GET_OSD_INFO_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_OSD_INFO_SIZE_OFFSET)
#define GET_AUDIO_OUT_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_AUDIO_OUT_ADDR_OFFSET)
#define GET_AUDIO_OUT_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_AUDIO_OUT_SIZE_OFFSET)
#define GET_METADATA_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_METADATA_ADDR_OFFSET)
#define GET_METADATA_SIZE_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_METADATA_SIZE_OFFSET)

/* RTCP sending buffer address */
#define GET_RTCP_DATA_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_RTCP_DATA_OFFSET)
/* Suspend/resume address */
#define SHMEM_TRAMPOLINE_OFFSET_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TOP_ADDR_SIZE + SHMEM_GR_IPCU_SIZE)
/* GR IPCU command buffer address */
#define GET_DISP_SETTING_ADDR_VBD_ACT		(SHMEM_TOP_ADDR_VBD_ACT + SHMEM_TOP_ADDR_SIZE)

