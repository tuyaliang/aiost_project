#ifndef MLB_FB_DISP_H
#define MLB_FB_DISP_H
#define D_MAX_COMMAND_NUM	(20)
#define D_MAX_COMMAND_WORD	(15)
/* >15*4+(20-4) +4 */
//#define D_MAX_COMMAND_BUFFER_SIZE (sizeof(t_request) * D_MAX_COMMAND_NUM / 4)
#define D_MAX_COMMAND_BUFFER_SIZE (200)
typedef enum {
	eGRTISIZE = 1,
	eGRTDSTA,
	eGRSCCTL,
	eGRRSZ,
	eGRISIZE,
	eGRDSTA,
	eGRHGA,
	eGRSA0,
	eGRSA,
	eGRAREN,
	eGRTRG,
	eGRIDT,
	eGRRST,
	eGRIPO,
} e_command_code;
typedef struct {
	unsigned int sequence_no;
	unsigned int data;
} t_ipcu_mail_box;

typedef union {
	unsigned int word;
	struct {
		unsigned int   command_num	:8;
		unsigned int   block_no		:8;
		unsigned int   gr_no		:8;
		unsigned int   reserve		:8;
	} bit;
} u_ipcu_command_head;
typedef union {
	unsigned int word;
	struct {
		unsigned int   size			:8;/* word's num */
		unsigned int   command_code	:8;
		unsigned int   reserve		:16;
	} bit;
} u_command_head;
typedef struct {
	u_command_head command_head;
	unsigned int command[D_MAX_COMMAND_WORD];

} t_request;
typedef struct {
	u_ipcu_command_head head;
	unsigned int command_offset[D_MAX_COMMAND_NUM];
	unsigned int buffer[D_MAX_COMMAND_BUFFER_SIZE];
} t_ipcu_command;
#endif

