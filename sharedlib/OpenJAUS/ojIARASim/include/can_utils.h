#ifndef CAN_UTILS_H_
#define CAN_UTILS_H_

#ifdef __cplusplus
extern "C"
{
#endif

	int init_can(char *intf_name);
	int send_frame(int sockfd, struct can_frame *frame);
	int recv_frame(int sockfd, struct can_frame *frame);
	void print_frame(struct can_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_UTILS_H_ */
