#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/of.h>

static int hardware_proc_show(struct seq_file *m, void *v)
{
	struct device_node *np;
	const char *id;
	int idlen;

	np = of_find_node_by_path("/hardware");
	if (!np)
		return -ENODEV;

	id = of_get_property(np, "id", &idlen);
	if (!id || strlen(id) > idlen)
		return -ENODEV;

	seq_printf(m, "%s\n", id);

	return 0;
}

static int hardware_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hardware_proc_show, NULL);
}

static const struct file_operations hardware_proc_fops = {
	.open		= hardware_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_hardware_init(void)
{
	proc_create("hardware", 0, NULL, &hardware_proc_fops);

	return 0;
}
fs_initcall(proc_hardware_init);
