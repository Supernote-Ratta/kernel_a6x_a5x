#ifndef __HTFYUN_AW87xxx_H
#define __HTFYUN_AW87xxx_H

#define REG_CHIPID		0x00
#define REG_SYSCTRL		0x01
#define REG_BATSAFE		0x02
#define REG_BSTOVR		0x03
#define REG_BSTVPR		0x04
#define REG_PAGR		0x05
#define REG_PAGC3OPR	0x06
#define REG_PAGC3PR		0x07
#define REG_PAGC2OPR	0x08
#define REG_PAGC2PR		0x09
#define REG_PAGC1PR		0x0A

int aw87xxx_set_audio_amplifier_enable(bool enabled);

#endif // __HTFYUN_AW87xxx_H

