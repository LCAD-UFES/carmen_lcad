
int init_can(char *intf_name);
int send_frame(int sockfd, struct can_frame *frame);
int recv_frame(int sockfd, struct can_frame *frame);
void print_frame(struct can_frame *frame);
