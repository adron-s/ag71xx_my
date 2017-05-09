/*
 *  Atheros AR71xx built-in ethernet mac driver
 *  Special support for the Atheros ar9344 switch chip
 *
 *  Copyright (C) 2017 Sergey Sergeev <adron@yapic.net>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include "ag71xx.h"
#include <linux/skbuff.h>

//размер Atheros header-а
#define AR9344_HEADER_LEN	2

/* для отладки кода осуществляющего добавление?удаление заголовка в skb */
#define skb_16b_ddd(skb) 																						\
	printk(KERN_DEBUG "%s(%s): (cl:%d,sh:%d)"													\
		"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:"											\
		"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x"												\
		"\n",																														\
		__func__, skb->dev ? skb->dev->name : "NULL",										\
		skb_cloned(skb), skb_shared(skb),																\
		skb->data[0], skb->data[1], skb->data[2], skb->data[3],					\
		skb->data[4], skb->data[5], skb->data[6], skb->data[7],					\
		skb->data[8], skb->data[9], skb->data[10], skb->data[11],				\
		skb->data[12], skb->data[13], skb->data[14], skb->data[15]);

void ag71xx_add_ar9344_header(struct sk_buff *skb, int port_mask){
 	u8 *r, *w; void *end;
	//проверка доступности свободного пространства для вмещения Atheros header-а
  if(unlikely(skb_headroom(skb) < AR9344_HEADER_LEN)){
		if(printk_ratelimit())
			printk(KERN_ERR "%s: critical error! no free space in skb headroom!\n",
						 __func__);
		return;
	}
	r = (void*)skb->data;
	__skb_push(skb, AR9344_HEADER_LEN);
	w = (void*)skb->data;
  end = (void*)skb->data + ETH_ALEN * 2;
  //проверка на вылет за пределы skb
  if(unlikely((void*)skb_tail_pointer(skb) < end + AR9344_HEADER_LEN)){
		if(printk_ratelimit())
			printk(KERN_ERR "%s: critical error! no free space in skb body!\n",
						 __func__);
		return;
	}
	//skb_16b_ddd(skb);
	//код который нас вызывает уже проверил skb на клониврованность!
	/* if(unlikely(skb_cloned(skb) || skb_shared(skb))){
		printk(KERN_ERR "%s: logic bug! skb is cloned!\n", __func__);
	} */
  /* перемещаем <-- [ src и dst мак адреса ] чтобы освободить
		 место для вставки Atheros Header-а */
	while(likely((void*)w < end)){
		//printk(KERN_INFO "%s: copy 0x%02x -> 0x%02x\n", __func__, *r, *w);
		*(w++) = *(r++);
	}
  //записываем Atheros header
	*(w++) = 0x80;
	*(w++) = 0x80 | port_mask;
	//printk(KERN_DEBUG "%s: port_mask = 0x%x\n", __func__, port_mask);
	//skb_16b_ddd(skb);
}

int ag71xx_remove_ar9344_header(struct sk_buff *skb, int pktlen, int *port_num){
	u8 *r = (void*)skb->data + ETH_ALEN * 2 - 1;
	u8 *w = r + AR9344_HEADER_LEN;
	/* запомним ~номер порта~ свитча с которого прилетела skb.
		 номер портка в сыром виде. не забудь для его очистки
		 применить соответствующую маску! */
	*port_num = *w;
  //проверка на вылет за пределы skb
  if(unlikely((void*)skb_tail_pointer(skb) <= (void*)w)){
		if(printk_ratelimit())
			printk(KERN_ERR "%s: critical error! no free space in skb body!\n",
						 __func__);
		return -EINVAL;
	}

	skb_16b_ddd(skb);
	/* так как build_skb не может порождать клонов то
		 и проверять на клоновость тут не нужно */
	/* if(unlikely(skb_cloned(skb) || skb_shared(skb))){
		printk(KERN_ERR "%s: logic bug! skb is cloned!\n", __func__);
	} */

  /* перемещаем [ src и dst мак адреса ] --> чтобы освободить
		 место для вставки Atheros Header-а */
	while(likely((void*)r >= (void*)skb->data)){
		//printk(KERN_INFO "%s: copy 0x%02x -> 0x%02x\n", __func__, *r, *w);
		*(w--) = *(r--);
	}
	//skb_16b_ddd(skb);
	skb_pull(skb, AR9344_HEADER_LEN);
	return 0;
}
