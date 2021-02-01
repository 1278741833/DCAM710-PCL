#ifndef __NET__
#define __NET__


class UDP_SERVER{
private:
	int send_fd;
	int recv_fd;
	struct sockaddr_in dest_agv;
	

	struct sockaddr_in addr_serv;
	
public:
	void sendInit(int port,int dirIP);
	void send(char *buf,int len);
	int  receive(char*,int&);
	void recvInit(int);

	int recv_len_max = 1024*4;
	char *revbuffer = NULL;
};





class TCP_SERVER{
public:
	bool long_connection = true;
	bool connected = false;
	int listenfd;
	int connfd;
	int client_addr=0;
	char *revbuffer=NULL;
	int  revbuf_lenmax = 1024;
	
	
public:
	int init(int,bool);
	void close_connection();
	int  rev_data(int&);
	void send_ackdata(char * ack_packet,int len);
	TCP_SERVER();
	~TCP_SERVER();

};

extern int monitor_ip;


int GetLocalIp(char *);

#endif
