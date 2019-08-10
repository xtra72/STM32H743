#ifndef GPIO_H_
#define GPIO_H_

void    GPIO_MASTER_setStatus(bool _ready);
bool    GPIO_SLAVE_isReady(void);
void    GPIO_AVDD_enable(void);
void    GPIO_AVDD_disable(void);
bool    GPIO_AVDD_isEnable(void);

#endif
