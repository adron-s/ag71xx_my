#ifndef DEBUG_H
#define DEBUG_H

/* байтовые преобразования. полезно для работы с ip адресами */
#define NIPQUAD(addr)          \
  ((unsigned char *)&addr)[0], \
  ((unsigned char *)&addr)[1], \
  ((unsigned char *)&addr)[2], \
  ((unsigned char *)&addr)[3]

#define HIPQUAD(addr)          \
  ((unsigned char *)&addr)[3], \
  ((unsigned char *)&addr)[2], \
  ((unsigned char *)&addr)[1], \
  ((unsigned char *)&addr)[0] 

#define HIPNETADDR(a, b, c, d) \
  (a & 0xFF) << 24   |         \
  (b & 0xFF) << 16   |         \
  (c & 0xFF) << 8    |         \
  (d & 0xFF)

#define SKB_PRINTD(skb)                                                                                      \
  printk(KERN_DEBUG "********************\n"                                                                 \
                    "%14s := 0x%x\n"                                                                         \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := %d(%d, d_l=%u)\n"                                                               \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := 0x%x(%d)\n"                                                                     \
                    "%14s := 0x%x (ip-0x800, ip_arp=0x806)\n"                                                \
                    "%14s := %d (HO-0,BR-1,MU-2,OT-3,OU-4)\n"                                                \
                    "%14s := (%d,%d,%d,%d)\n"                                                                \
                    "%14s := %d\n"                                                                           \
                    "%14s := %s\n"                                                                           \
                    "--------------------\n",                                                                \
                    "head", (u32)skb->head,                                                                  \
                    "data", (u32)skb->data, skb->data - skb->head,                                           \
                    "tail", (u32)skb_tail_pointer(skb), skb_tail_pointer(skb) - skb->head,                   \
                    "end", (u32)skb_end_pointer(skb), skb_end_pointer(skb) - skb->head,                      \
                    "len", (u32)skb->len, skb_tail_pointer(skb) - skb->data, skb->data_len,                  \
                    "mac_h(eth_h)", (u32)skb_mac_header(skb), skb_mac_header(skb) - skb->head,               \
                    "net_h(ip_h)", (u32)skb_network_header(skb), skb_network_header(skb) - skb->head,        \
                    "tr_h(udp_h)", (u32)skb_transport_header(skb), skb_transport_header(skb) - skb->head,    \
                    "prot", ntohs(skb->protocol),                                                            \
                    "p_type", skb->pkt_type,                                                                 \
                    "(sh,cl,clw,nl)", skb_shared(skb), skb_cloned(skb),                                      \
                                      skb_clone_writable(skb, 0), skb_is_nonlinear(skb),                     \
                    "nr_fr", skb_shinfo(skb)->nr_frags,                                                      \
                    "dev", (!skb->dev?"NULL":skb->dev->name)                            );

#define IPH_PRINTD(iph) \
  printk(KERN_DEBUG "********************\n"                                     \
                    "s_ip  =  %u.%u.%u.%u\n"                                     \
                    "d_ip  =  %u.%u.%u.%u\n"                                     \
                    "total_l =  %d\n"                                            \
                    "id =     0x%x\n"                                            \
                    "frag_off = 0x%x\n"                                          \
                    "prot =  %d (ip-0,1-icmp,ipip-4,tcp-6,egp-8,udp-17)\n"       \
                    "check =  0x%x\n"                                            \
                    "--------------------\n",                                    \
                     NIPQUAD(iph->saddr),                                        \
                     NIPQUAD(iph->daddr),                                        \
                     htons(iph->tot_len),                                        \
                     htons(iph->id),                                             \
                     htons(iph->frag_off),                                       \
                     iph->protocol,                                              \
                     iph->check                                                  );

#define UDPH_PRINTD(iph, udph) \
  printk(KERN_DEBUG "********************\n"                                     \
                    "s_ip  =  %u.%u.%u.%u:%u\n"                                  \
                    "d_ip  =  %u.%u.%u.%u:%u\n"                                  \
                    "h_t_l =  %d\n"                                              \
                    "check =  0x%x\n"                                            \
                    "--------------------\n",                                    \
                     NIPQUAD(iph->saddr),                                        \
                     htons(udph->source),                                        \
                     NIPQUAD(iph->daddr),                                        \
                     htons(udph->dest),                                          \
                     htons(udph->len),                                           \
                     udph->check                                                 );

#define IN_IRQ_AND_CONTENT_PRINTD()                        \
  printk(KERN_DEBUG "summary of content state(fn %s):\n"   \
                    "smp_processor_id() - cpu .. %d\n"     \
                    "in_atomic() ... %d\n"                 \
                    "in_interrupt() ... %lu\n"    \
                    "in_irq() ... %lu\n"     \
                    "in_softirq() ... %lu\n" \
                    "in_serving_softirq() ... %lu\n"       \
                    "irqs_disabled() ... %d\n",            \
                    __func__,                              \
                    smp_processor_id(),                    \
                    in_atomic(),                           \
                    in_interrupt(),                        \
                    in_irq(),                              \
                    in_softirq(),                          \
                    in_serving_softirq(),                  \
                    irqs_disabled()                       );


#ifdef DEBUG
#define PRINTD(format, args...) printk(KERN_DEBUG "%lu/<ukt[%s]>" format, jiffies, __func__, ##args);
#define PRINTD_RATELIMIT(format, args...) if(printk_ratelimit()) PRINTD(format, ##args)
#define PRINTD2(format, args...) printk(KERN_DEBUG "%lu/<ukt[%s]:tun:%s[%u])>" format, jiffies, __func__, tun->name, tun->num, ##args);
#define PRINT_SCHD(format, ring, args...) printk(KERN_DEBUG "%lu/<ukt[%s]/sub_chan(ring_%d[%d])" \
                                                 format, jiffies, __func__, ring, sub_chan->num, ##args);
#define PRINT_SCHD2(format, ring, args...) printk(KERN_DEBUG "%lu/<ukt[%s]:tun:%s[%u]/sub_chan(ring_%d[%d])>" \
                                                  format, jiffies, __func__, tun->name, tun->num, ring, sub_chan->num, ##args);
#define GET_SUB_CHAN_INFO(res_var_name)                                \
  char res_var_name[255];                                              \
  sprintf(res_var_name, "me[%u.%u.%u.%u:%u]-->peer[%u.%u.%u.%u:%u]",   \
          NIPQUAD(sub_chan->our_ip), htons(sub_chan->our_port),        \
          NIPQUAD(sub_chan->peer_ip), htons(sub_chan->peer_port));
#else
#define PRINTD(...)
#define PRINTD_RATELIMIT(...)
#define PRINTD2(...)
#define PRINT_SCHD(...)
#define PRINT_SCHD2(...)
#define GET_SUB_CHAN_INFO(...)
#endif

#endif /* DEBUG_H */
