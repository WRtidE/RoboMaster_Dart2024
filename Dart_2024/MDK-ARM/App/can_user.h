#ifndef CAN_USER_H
#define CAN_USER_H

#include "struct_typedef.h"

/**
  * @brief          ���͵�����Ƶ�����������can1��
  * @param[in]      id_range: (1��0)id_range ȡֵΪ0ʱ0x200(id<5),idȡֵΪ1ʱ0x1ff(id>=5);
  * @param[in]      m1: ������Ƶ���
  * @param[in]      m2: ������Ƶ���
  * @param[in]      m3: ������Ƶ���
  * @param[in]      m4: ������Ƶ���
  * @retval         none
  */
void can1_cmd_motor(uint8_t id_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void can2_cmd_motor(uint8_t id_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif
