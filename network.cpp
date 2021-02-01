#include "main.h"

MatrixXf transFromAngToAxis(const MatrixXf& s_data)
{
	MatrixXf t_data(s_data.rows(),2);
	t_data.col(U_FEA_X) = s_data.col(S_FEA_DIST).array()* cos(s_data.col(S_FEA_ANG).array()*A2R);
	t_data.col(U_FEA_Y) = s_data.col(S_FEA_DIST).array()* sin(s_data.col(S_FEA_ANG).array()*A2R);
	return t_data;
}



NETWORK::NETWORK()
{
	bufsize_max = 1024*1024*10;
	tcp_server.client_addr = -1;
	udp_thread_start = false;
}



int NETWORK::udpPackingPacket(char *buf)
{
	MatrixXf  u_data =	 transFromAngToAxis(sys->r2000.sourcedata.m);
	buf[0] = 0x20;
	buf[1] = 0x03;
	int buf_size = packageFrameBuf(buf+2,u_data) + 2;
	
	return buf_size;
}

int NETWORK::udpSendRun()
{

	while(tcp_server.client_addr <=0)
	{
		util::sleep(1000);
	}
	udp_server.sendInit(sys->param.net_tcp_port,tcp_server.client_addr);
	
	while(true)
	{
		util::sleep(100);

		udp_pocket = new char[bufsize_max];
		int send_num = udpPackingPacket(udp_pocket);
		udp_server.send(udp_pocket, send_num);

		delete[] udp_pocket;
 	}
}




bool NETWORK::tcpCmdHandle(const TCP_DATA& data)
{
	if(tcp_ack.buf !=NULL)
	{
		delete[] tcp_ack.buf;
	}
	tcp_ack.buf = new char[bufsize_max];
	tcp_ack.size = 0;

	tcp_ack.buf[0]= 0x01;
	tcp_ack.buf[1]= 0x04;
	tcp_ack.size+=2;
	switch(data.cmd)
	{
		case CMD_INIT: 
		{
			if(udp_thread_start == false)
			{
				cout<<"[START THREAD] --> NETWORK UDP SEND"<<endl;
				thread NetUDPSendthread(&NETWORK::udpSendRun, this);
			    NetUDPSendthread.detach();	
				udp_thread_start = true;
			}
			//last_data = transFromAngToAxis(sys->r2000.sourcedata.m);
		// 	int cmd_init = CMD_INIT;memcpy(tcp_ack.buf+tcp_ack.size,&cmd_init,sizeof(cmd_init));tcp_ack.size+=4;


		// 	// int buf_size = packageFrameBuf(tcp_ack.buf+6,sys->r2000.sourcedata.m) + 6;//result
		// 	int success = SUCCESS;memcpy(tcp_ack.buf+tcp_ack.size,&success,sizeof(success));tcp_ack.size+=4;

		// //	tcp_ack.size += packageFrameBuf(tcp_ack.buf+tcp_ack.size,last_data);

		// 	uint16_t crc_checsum = getbufferCrc(tcp_ack.buf,tcp_ack.size);
		// 	memcpy(tcp_ack.buf+tcp_ack.size,&(crc_checsum),sizeof(uint16_t)); tcp_ack.size+=sizeof(uint16_t);

			break;
		}
		
	}

	tcp_server.send_ackdata(tcp_ack.buf, tcp_ack.size);

	
	delete[] tcp_ack.buf;
	tcp_ack.buf = NULL;
	
	
}



int NETWORK::packageFrameBuf(char *buf,const MatrixXf& s_data)
{
	
	int offset = 0;

	int pnt_num = s_data.rows();
	
	memcpy(buf+offset,&pnt_num,sizeof(int)); offset+=sizeof(int);

	if(pnt_num > 0)
	{
		MatrixXf  u_data =	 transFromAngToAxis(s_data);
		for(int i=0;i<pnt_num;i++)
		{
			memcpy(buf+offset,&(u_data(i,U_FEA_X)),sizeof(float)); offset+=sizeof(float);
			memcpy(buf+offset,&(u_data(i,U_FEA_Y)),sizeof(float)); offset+=sizeof(float);
		}
		//uint16_t crc_checsum = getbufferCrc(buf,offset);
		
		//memcpy(buf+offset,&(crc_checsum),sizeof(uint16_t)); offset+=sizeof(uint16_t);
	}

	return offset;
}

int NETWORK::tcpServerRun()
{

	tcp_server.init(sys->param.net_tcp_port,false);
	
	while(true)
	{
		util::sleep(1000);
		
		if(tcp_server.long_connection == false && tcp_server.connected == true)
			tcp_server.close_connection();

		int get_addr = 0;
		int getbuflen = tcp_server.rev_data(get_addr);
		
		if((0xff&tcp_server.revbuffer[0])== 0x00 && (0xff&tcp_server.revbuffer[1]) == 0x03)
		{
			cout << "do"<<endl;
			tcp_server.client_addr = get_addr;
			memcpy(&tcp_data,tcp_server.revbuffer,sizeof(TCP_DATA));
			tcpCmdHandle(tcp_data);
		}
		
	}
	return 0;
}



void NETWORK::ThreadSimNet()
{

	cout<<"[START THREAD] --> NETWORK TCP SERVER"<<endl;
	thread NetTCPthread(&NETWORK::tcpServerRun, this);
    NetTCPthread.detach();	

}

