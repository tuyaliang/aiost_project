/*
 * these are the  structs and constants that define the transport of the
 * linux filesystem api
 */

#define IPCUFS_MAX_NAME_LEN 256

#define IPCUFS_REMOTE_TYPE_DIR 1

#define IPCUFS_MAX_FILESYSTEM_NAME_LEN 8

enum ipcufs_action {
	IFSA_IGET,			/* return given inode */
	IFSA_READPAGE,			/* fill a 4K page with file payload */
	IFSA_READDIR,			/* get the next dir entry */
	IFSA_LOOKUP,			/* lookup a file from an inode */
	IFSA_WRITEPAGE,			/* store payload from a 4K page */
	IFSA_ICREATE,			/* set up a new inode */
	IFSA_IDELETE,			/* delete the inode */
	IFSA_IRENAME,			/* rename the inode */
	IFSA_MOUNTING,			/* becoming mounted on linux side */
	IFSA_UMOUNTING,			/* becoming unmounted on linux side */
};

struct ipcufs_u_iget {
	u64 inode;			/* filled in on entry */
	u64 inode_length;		/* filled in on return */
	u64 mtime_s;			/* filled in on return */
	u64 atime_s;			/* filled in on return */
	u64 ctime_s;			/* filled in on return */
	u32 mtime_ns;			/* filled in on return */
	u32 atime_ns;			/* filled in on return */
	u32 ctime_ns;			/* filled in on return */
	u16 type;			/* filled in on return */
} __packed;

struct ipcufs_page_entry
{
	u32 pa;
	u32 len;
};

#define MAX_IPCUFS_PAGES_AT_ONCE 64

struct ipcufs_u_page {
	u64 inode;			/* filled in on entry */
	u32 count_pages;
	u64 pa; // for write
	u64 pos; // for write
	u32 len; // for write
	struct ipcufs_page_entry p[MAX_IPCUFS_PAGES_AT_ONCE];
			/* filled in on entry */
} __packed;

struct ipcufs_u_readdir {
	u64 inode;			/* filled in on entry */
	u64 ctxpos;			/* filled in on entry */
	char name[IPCUFS_MAX_NAME_LEN];	/* filled in on return */
	u16 name_len;			/* filled in on return */
	u16 type;			/* filled in on return */
	u64 i_sibling;			/* filled in on return */
} __packed;

struct ipcufs_u_lookup {
	u64 inode_in;			/* filled in on entry */
	char name[IPCUFS_MAX_NAME_LEN];	/* filled in on entry */
	u16 name_len;			/* filled in on entry */
	u64 inode_out;			/* filled in on return */
} __packed;

struct ipcufs_u_icreate {
	struct ipcufs_u_iget idata;
	char name[IPCUFS_MAX_NAME_LEN];
	u16 name_len;
	u64 i_parent;
};

struct ipcufs_u_idel {
	u64 inode;			/* filled in on entry */
	u64 inode_parent;		/* filled in on entry */
};

struct ipcufs_u_iren {
	u64 inode;
	char name[IPCUFS_MAX_NAME_LEN];
	u16 name_len;
};

struct ipcufs_transport {
	char fs_name[IPCUFS_MAX_FILESYSTEM_NAME_LEN]; /* filled on entry */
	enum ipcufs_action action;
	u32 result;			/* filled in on return 0 = OK */
	union {
		struct ipcufs_u_iget		iget;
		struct ipcufs_u_page		page;
		struct ipcufs_u_readdir		rdir;
		struct ipcufs_u_lookup		lookup;
		struct ipcufs_u_icreate		icreate;
		struct ipcufs_u_idel		idel;
		struct ipcufs_u_iren		iren;
	} u;
};
