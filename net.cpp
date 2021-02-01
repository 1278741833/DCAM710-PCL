#include "main.h"



void UDP_SERVER::sendInit(int port,int dirIP)
{
	
	bzero(&dest_agv, sizeof(dest_agv)); 
	bzero( &addr_serv, sizeof( addr_serv) );

	dest_agv.sin_family = AF_INET;
	dest_agv.sin_port = htons(port);


	
	dest_agv.sin_addr.s_addr = (dirIP);


	if((send_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
	{
		perror("Socketagv");
		//exit(errno);
	}
}
void UDP_SERVER::recvInit(int port)
{
	if(revbuffer == NULL)
	{
		revbuffer = new char[recv_len_max];
	}

	recv_fd = socket(AF_INET, SOCK_DGRAM, 0);
	
	addr_serv.sin_family = AF_INET;
	addr_serv.sin_port = htons(port);
	addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);

	if(bind(recv_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)  
	{
		LOG_ERROR("SIMNET")<<"recvInit bind fail";
	exit(0);
	}
}
void UDP_SERVER::send(char *buf,int len)
{
	
	sendto(send_fd, buf, len, 0, (struct sockaddr*)&dest_agv, sizeof(dest_agv));
}

int UDP_SERVER::receive(char * buf,int& senderip)
{

	memset(buf,0,recv_len_max);
	
	socklen_t Addr_len = sizeof(struct sockaddr_in);
	int recv_len = recvfrom(recv_fd, buf, recv_len_max, 0,(struct sockaddr *)&addr_serv,&Addr_len);

	senderip = addr_serv.sin_addr.s_addr;
	//cout<<"addr:"<<addr_serv.sin_addr.s_addr<<endl;

	return recv_len;

}







//////////////////////////////////////////////////////

TCP_SERVER::TCP_SERVER()
{

	if(NULL == revbuffer)
	{
		revbuffer = new char[revbuf_lenmax];
	}
}
TCP_SERVER::~TCP_SERVER()
{
	if(NULL != revbuffer)
	{
		delete[] revbuffer;
		revbuffer = NULL;
	}


}


int TCP_SERVER::init(int port,bool longconnection)
{
	long_connection = longconnection;
	struct sockaddr_in server_addr;

	if((listenfd = (socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))) == -1)
	{
		LOG_ERROR("NetBase")<<"Create tcp server local socket failed!"<<endl;
		return 1;
	}
#if 1
	struct timeval timeout = {5, 0};
	setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

	int reuseon = 1;
	if(setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuseon, sizeof(reuseon))<0)
	{
		LOG_ERROR("NetBase")<<"Tcp server set address reuse failed! errno:"<<errno<<":"<<strerror(errno)<<endl;
	}
#endif
	
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(port);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr(localIP);

	if(bind(listenfd, (struct sockaddr*)&server_addr, sizeof(server_addr))==-1)
	{
		LOG_ERROR("NetBase")<<"TCP server:"<<server_addr.sin_addr.s_addr<<" bind failed. errno:"<<errno<<":"<<strerror(errno)<<endl;
		close(listenfd);
		return 2;
	}
	else
	{
		LOG_WARN("NetBase")<<"TCP server band success,port:"<<  port <<endl;
	}

	if (listen(listenfd, 128) < 0) {
		LOG_ERROR("NetBase")<<"TCP server listen on server socket error. errno:"<<errno<<":"<<strerror(errno)<<endl;
		close(listenfd);
		return 3;
	}else{
		LOG_INFO("NetBase")<<"TCP server:"<<server_addr.sin_addr.s_addr<<":"<<port<<"  listening ."<<endl;
	}
	return 0;
}


void TCP_SERVER::close_connection()
{
//	printf("###############TCP SERVER close connection\n");
	close(connfd);
	connected = false;

}

int TCP_SERVER::rev_data(int &cltaddr)
{
   //senderip = addr_serv.sin_addr.s_addr;

   
	struct sockaddr_in client_addr;
	socklen_t length = sizeof(client_addr);
	if(connected == false || long_connection == false)
	{
		if(-1 ==(connfd = accept(listenfd, (struct sockaddr*)&(client_addr), &length)))
		{
			printf("connfd:%d\n",connfd);
			return -1;
		}

		else
		{
			connected = true;
			if(cltaddr == 0)
				cltaddr = client_addr.sin_addr.s_addr;
			//cout<<((client_addr.sin_addr.s_addr))<<endl;;
		}
	  
	}
	
	memset(revbuffer, 0,revbuf_lenmax);
	int getbuflen  = recv(connfd, revbuffer, revbuf_lenmax, 0);
	
   
	if(getbuflen <=0)
	{
		if(errno ==	11)
		{
		   LOG_WARN("NetBase")<<"TCP server receive data timeout on port:"<<endl;
		}
		else
		{
		   LOG_ERROR("NetBase")<<"TCP server recieve data failed! port:"<<" errno:"<<errno<<":"<<strerror(errno)<<endl;
		}
		if( long_connection == false)
		{
		   close_connection();

		}

		return -1;
	}

	return getbuflen;


}

void TCP_SERVER::send_ackdata(char * ack_packet,int len)
{
	//tcpdata.clientIP = inet_ntoa(tcpdata.remoteaddr.sin_addr);

	send(connfd,ack_packet,len,0);
}

int  GetLocalIp(char* outip)
{
    int i=0;
    int sockfd;
    struct ifconf ifconf;
    char temp_buf[512];
    struct ifreq *ifreq;
    char* ip;
    ifconf.ifc_len = 512;
    ifconf.ifc_buf = temp_buf;

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0))<0)
    {
        return 1;
    }
    ioctl(sockfd, SIOCGIFCONF, &ifconf);    //获取所有接口信息
    close(sockfd);
    //接下来一个一个的获取IP地址
    ifreq = (struct ifreq*)temp_buf;

    for(i=(ifconf.ifc_len/sizeof(struct ifreq)); i>0; i--)
    {
        ip = inet_ntoa(((struct sockaddr_in*)&(ifreq->ifr_addr))->sin_addr);
        //排除127.0.0.1，继续下一个
        if(strcmp(ip,"127.0.0.1")==0)
        {
            ifreq++;
            continue;
        }
        strcpy(outip,ip);
        return 0;
    }
    return 2;
}


