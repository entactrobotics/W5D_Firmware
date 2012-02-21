// --------------------------------------------------------------------------------------
// Module     : UDP
// Author     : R. Leslie (June 2011)
// --------------------------------------------------------------------------------------
// Entact Robotics inc. www.entactrobotics.com
// --------------------------------------------------------------------------------------
#ifndef UDP_H
#define UDP_H

#define UDP_PORT        (55556)

int udp_init(void);
int udp_uninit(void);
int udp_send(void *buf, int len);

#endif // UDP_H

