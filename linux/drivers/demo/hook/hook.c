#include <linux/kernel.h>
#include <linux/kprobes.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <net/tcp.h>

#define OPTSIZE	5
// saved_op������ת��ԭʼ������ָ��
char saved_op[OPTSIZE] = {0};

// jump_op������ת��hook������ָ��
char jump_op[OPTSIZE] = {0};

static unsigned int (*ptr_orig_conntrack_in)(const struct nf_hook_ops *ops, struct sk_buff *skb, const struct net_device *in, const struct net_device *out, const struct nf_hook_state *state);
static unsigned int (*ptr_ipv4_conntrack_in)(const struct nf_hook_ops *ops, struct sk_buff *skb, const struct net_device *in, const struct net_device *out, const struct nf_hook_state *state);

// stub���������ս��ᱻ����ָ���buffer���ǵ�
static unsigned int stub_ipv4_conntrack_in(const struct nf_hook_ops *ops, struct sk_buff *skb, const struct net_device *in, const struct net_device *out, const struct nf_hook_state *state)
{
	printk("hook stub conntrack\n");
	return 0;
}

// �������ǵ�hook���������ں��ڵ���ipv4_conntrack_in��ʱ�򣬽��ᵽ�����������
static unsigned int hook_ipv4_conntrack_in(const struct nf_hook_ops *ops, struct sk_buff *skb, const struct net_device *in, const struct net_device *out, const struct nf_hook_state *state)
{
	printk("hook conntrack\n");
	// ������ӡһ����Ϣ�󣬵���ԭʼ������
	return ptr_orig_conntrack_in(ops, skb, in, out, state);
}

static void *(*ptr_poke_smp)(void *addr, const void *opcode, size_t len);
static __init int hook_conn_init(void)
{
	s32 hook_offset, orig_offset;

	// ���poke������ɵľ�����ӳ�䣬дtext�ε���
	ptr_poke_smp = kallsyms_lookup_name("text_poke_smp");
	if (!ptr_poke_smp) {
		printk("%s LINE = %d\n",__func__,__LINE__);
		return -1;
	}

	// �ţ����Ǿ���Ҫhookסipv4_conntrack_in������Ҫ���ҵ�����
	ptr_ipv4_conntrack_in = kallsyms_lookup_name("ipv4_conntrack_in");
	if (!ptr_ipv4_conntrack_in) {
		printk("%s LINE = %d\n",__func__,__LINE__);
		return -1;
	}

	// ��һ���ֽڵ�Ȼ��jump
	jump_op[0] = 0xe9;
	// ����Ŀ��hook��������ǰλ�õ����ƫ��
	hook_offset = (s32)((long)hook_ipv4_conntrack_in - (long)ptr_ipv4_conntrack_in - OPTSIZE);
	// ����4���ֽ�Ϊһ�����ƫ��
	(*(s32*)(&jump_op[1])) = hook_offset;

	// ��ʵ�ϣ����ǲ�û�б���ԭʼipv4_conntrack_in������ͷ����ָ�
	// ����ֱ��jmp����5��ָ����ָ���Ӧ��ͼ��Ӧ����ָ��buffer��û
	// ��old inst��ֱ�Ӿ���jmp y�ˣ�Ϊʲô�أ�����ϸ˵��
	saved_op[0] = 0xe9;
	// ����Ŀ��ԭʼ������Ҫִ�е�λ�õ���ǰλ�õ�ƫ��
	orig_offset = (s32)((long)ptr_ipv4_conntrack_in + OPTSIZE - ((long)stub_ipv4_conntrack_in + OPTSIZE));
	(*(s32*)(&saved_op[1])) = orig_offset;


	get_online_cpus();
	// �滻������
	ptr_poke_smp(stub_ipv4_conntrack_in, saved_op, OPTSIZE);
	ptr_orig_conntrack_in = stub_ipv4_conntrack_in;
	barrier();
	ptr_poke_smp(ptr_ipv4_conntrack_in, jump_op, OPTSIZE);
	put_online_cpus();

	return 0;
}
module_init(hook_conn_init);

static __exit void hook_conn_exit(void)
{
	get_online_cpus();
	ptr_poke_smp(ptr_ipv4_conntrack_in, saved_op, OPTSIZE);
	ptr_poke_smp(stub_ipv4_conntrack_in, jump_op, OPTSIZE);
	barrier();
	put_online_cpus();
}
module_exit(hook_conn_exit);

MODULE_DESCRIPTION("hook test");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
