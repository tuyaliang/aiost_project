#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/random.h>

static int cmdline_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", saved_command_line);
	return 0;
}

static int cmdline_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cmdline_proc_show, NULL);
}

static const struct file_operations cmdline_proc_fops = {
	.open		= cmdline_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#ifdef CONFIG_MBOX_F_IPCU
extern void ipc_send(int sender);

static int ipcu_proc_show(struct seq_file *m, void *v)
{
	unsigned long sender;

	prandom_bytes(&sender, sizeof(sender));
	sender %= 8;
	ipc_send(sender);

	return 0;
}

static int ipcu_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ipcu_proc_show, NULL);
}

static const struct file_operations ipcu_proc_fops = {
	.open		= ipcu_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

static int __init proc_cmdline_init(void)
{
	proc_create("cmdline", 0, NULL, &cmdline_proc_fops);
#ifdef CONFIG_MBOX_F_IPCU
	proc_create("ipcu", 0, NULL, &ipcu_proc_fops);
#endif
	return 0;
}
fs_initcall(proc_cmdline_init);

