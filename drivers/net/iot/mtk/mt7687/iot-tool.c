#include  <stdio.h>
#include  <stdlib.h>
#include  <string.h>
#include  <fcntl.h>
#include  <errno.h>
#include  <sys/ioctl.h>
#include  <sys/types.h>
#include  <sys/socket.h>
#include  <linux/if.h>

#include <stddef.h>             /* offsetof */
#include <netinet/in.h>
#if __GLIBC__ >=2 && __GLIBC_MINOR >= 1
#include <netpacket/packet.h>
#include <net/ethernet.h>
#else
#include <asm/types.h>
#include <linux/if_ether.h>
#endif


#include <getopt.h>

#include "iot.h"



typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned char u8;

#include  <linux/sockios.h>
#include  <linux/ethtool.h>

void
Perror(cmd)
	const char *cmd;
{
	switch (errno) {

	case ENXIO:
		errx(1, "%s: no such interface", cmd);
		break;

	case EPERM:
		errx(1, "%s: permission denied", cmd);
		break;

	default:
		err(1, "%s", cmd);
	}
}

void
setifflags(int fd, const char *if_name, int value)
{
	struct ifreq ifr;
	int	flags;

	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);

 	if (ioctl(fd, SIOCGIFFLAGS, (caddr_t)&ifr) < 0) {
 		Perror("ioctl (SIOCGIFFLAGS)");
 		exit(1);
 	}

 	flags = ifr.ifr_flags;

	if (value < 0) {
		value = -value;
		flags &= ~value;
	} else
		flags |= value;
	ifr.ifr_flags = flags;
	if (ioctl(fd, SIOCSIFFLAGS, (caddr_t)&ifr) < 0)
		Perror(if_name);
}


// if_name like "ath0", "eth0". Notice: call this function
// need root privilege.
// return value:
// -1 -- error , details can check errno
// 1 -- interface link up
// 0 -- interface link down.
int iot_get_netlink_status(int fd, const char *if_name)
{
    
    struct ifreq ifr;
    struct ethtool_value edata;
    
    edata.cmd = ETHTOOL_GLINK;
    edata.data = 0;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;

    if(ioctl( fd, SIOCETHTOOL, &ifr ) == -1)
    {
	close(fd);
	return -1;
    }

    return edata.data;
}

int iot_get_info(int fd, const char *if_name, mt7687_dev_info_t* info)
{
    
    struct ifreq ifr;
    struct ethtool_value edata;
    
    edata.cmd = 0;
    edata.data =(u32) info;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;

    if(ioctl( fd, SIOCG_DEVINFO, &ifr ) == -1)
    {
	return -1;
    }

    return 0;
}


int iot_set_wifi_setting(int fd, const char *if_name, iot_wifi_setting_t *wifisetting)
{
    struct ifreq ifr;
    struct ethtool_value edata;
    int ret;    
    
    edata.cmd = IOT_SET_WIFI_SETTING;
    edata.data =(u32) wifisetting;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;

    if(ret=ioctl( fd, SIOC_IOT_WIFI, &ifr ) == -1)
    {
	return -1;
    }    
    
    return ret;
}

int iot_get_wifi_setting(int fd, const char *if_name , iot_wifi_setting_t *iot_wifi_setting_t)
{
    struct ifreq ifr;
    struct ethtool_value edata;
    int ret;
    
    edata.cmd = IOT_GET_WIFI_SETTING;
    edata.data = (__u32) iot_wifi_setting_t;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;

    if(ret=ioctl( fd, SIOC_IOT_WIFI, &ifr ) == -1)
    {
	return ret;
    }    

    printf("IoT WIFI Info:\n");    
    printf("wifi SSID: %s\n",iot_wifi_setting_t->ssid);
    printf("wifi Password: %s\n",iot_wifi_setting_t->psk);
    printf("wifi auth: %d\n",iot_wifi_setting_t->auth);
    printf("wifi encrypt: %d\n",iot_wifi_setting_t->encrypt);
	
//    printf("wifi Reconnect: %d\n",iot_wifi_setting_t->reconnect);
      
    return 0;
}


int iot_set_low_power(int fd, const char *if_name)
{
    struct ifreq ifr;
    struct ethtool_value edata;
    int ret;    
    
    edata.cmd = IOT_SET_IOT_STANDBY;
    edata.data =(u32) 0;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);
    ifr.ifr_data = (char *) &edata;

    if(ret=ioctl( fd, SIOCS_POWERMODE, &ifr ) == -1)
    {
	return -1;
    }    
    
    return ret;
}


#define IFNAME "eth0:2"
#define HOST "192.168.1.204"
#define ifreq_offsetof(x)  offsetof(struct ifreq, x)

int iot_set_ifaddr(int fd, const char *if_name, 	__be32 ipaddr ) {

        struct ifreq ifr;
        struct sockaddr_in sai;
        int sockfd;                     /* socket fd we use to manipulate stuff with */
        int selector;
        unsigned char mask;

        char *p;

        /* get interface name */
	memset(&ifr, 0, sizeof(ifr));
	strncpy(ifr.ifr_name, if_name, sizeof(ifr.ifr_name) - 1);

        memset(&sai, 0, sizeof(struct sockaddr));
        sai.sin_family = AF_INET;
        sai.sin_port = 0;

        sai.sin_addr.s_addr = ipaddr;//inet_addr(HOST);

        p = (char *) &sai;
        memcpy( (((char *)&ifr + ifreq_offsetof(ifr_addr) )),
                        p, sizeof(struct sockaddr));

        ioctl(fd, SIOCSIFADDR, &ifr);
        ioctl(fd, SIOCGIFFLAGS, &ifr);

        ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
        // ifr.ifr_flags &= ~selector;  // unset something

        ioctl(fd, SIOCSIFFLAGS, &ifr);
      //  close(sockfd);
        return 0;
}


void help(char *progname)
{
  fprintf(stderr, "------------------------------------------------------------------\n");
  fprintf(stderr, "Usage: %s\n" \
                  "Available options are\n"\
                  " [-h | --help ]........: display this help\n" \
                  " [-up].......: Interface up.\n" \
                  " [-m | --opmode ]..: STA/AP Mode. 0: STA(default)  \n" \
                  " [-a | --auth ]......: Auth mode. 0: open 9:wpa/wpa2\n" \
                  " [-e | --encrypt ].........: Encryp type. 0: wep 1: open 8: wpa \n" \
                  " [-s | --ssid ].......: Connect to SSID . \n" \
                  " [-p | --psk ].......: Password \n" \
                  " [-r | --reconnect ].......: Reconnect to AP w/ new config.\n" \
                  " [-i | --info ].......: Get IoT device info and set to net I/F. ex: ip address.\n" \
                  " [-I | --IF].......: Net device interface to set.(default: eth0)\n" \
                  " [-L ].......: Enter Low power mode\n" \
                  " [-v | --version ].....: display version information\n" \
                  , progname);
  
  fprintf(stderr, "------------------------------------------------------------------\n");

}


int main(int argc, char* argv[])
{
	char* pIFname = "eth0";
	int skfd;
	int ret;
	iot_wifi_setting_t wifi = {.auth = 0, .encrypt=1};
	mt7687_dev_info_t info;
	int bIsGetInfo = 0;
	int linkstatsus;
	int lowpower = 0;
	
	while(1) {
	    int option_index = 0, c=0;
	    static struct option long_options[] = 
	    {
	      {"h", no_argument, 0, 0},
	      {"help", no_argument, 0, 0},

	      {"m", required_argument, 0, 0},
	      {"mode", required_argument, 0, 0},

	      {"a", required_argument, 0, 0},
	      {"auth", required_argument, 0, 0},

	      {"e", required_argument, 0, 0},
	      {"encrypt", required_argument, 0, 0},
	      
	      {"s", required_argument, 0, 0},
	      {"ssid", required_argument, 0, 0},
	      
	      {"p", required_argument, 0, 0},		// 10
	      {"psk", required_argument, 0, 0},

	      {"r", no_argument, 0, 0},
	      {"reconnect", no_argument, 0, 0},

	      {"i", no_argument, 0, 0},
	      {"info", no_argument, 0, 0},
	      
	      {"I", required_argument, 0, 0},
	      {"IF", required_argument, 0, 0},

	      {"L", no_argument, 0, 0},     
	      
	      {0, 0, 0, 0}
	    };

	    c = getopt_long_only(argc, argv, "", long_options, &option_index);

	    /* no more options to parse */
	    if (c == -1) 
			break;

	    /* unrecognized option */
	    if(c=='?'){ help(argv[0]); return 0; }

	    printf("option_index = %d.\n",option_index);
		
	    switch (option_index) {
	      /* h, help */
	      case 0:
	      case 1:
	        help(argv[0]);
	        return 0;
	        break;

	      /* m, mode */
	      case 2:
	      case 3:
	        wifi.opmode = 0;// atoi(optarg);
	        break;

	      /* a, auth */
	      case 4:
	      case 5:
	        wifi.auth = atoi(optarg);
	        break;		
		/* e, encrypt */
	      case 6:
	      case 7:
	        wifi.encrypt = atoi(optarg);
	        break;	

		/* s, ssid */
	      case 8:
	      case 9:
	        strcpy(wifi.ssid,optarg);
	        break;	
			
		/* p, psk */
	      case 10:
	      case 11:
	        strcpy(wifi.psk ,optarg);
	        break;		


		/* r, reconnect */
	      case 12:
	      case 13:
			wifi.reconnect = 1;//atoi(optarg);
			break;

		/* i, info */	
		case 14:
		case 15:			
	        bIsGetInfo = 1;
	        break;			

		/* I, IF*/
	      case 16:
	      case 17:
			pIFname = (char*)strdup(optarg);
	        break;				

	      case 18:
			lowpower = 1;
	        break;	
			
	      default:
	        help(argv[0]);
	        return 0;
	    }
	  }

	if(getuid() != 0)
	{
		fprintf(stderr, "Netlink Status Check Need Root Power.\n");
		return 1;
	}

	printf("Interface: %s\n ",pIFname);

        /* Create a channel to the NET kernel. */

	if (( skfd = socket( AF_INET, SOCK_DGRAM, 0 )) == 0)
		return -1;

	if(lowpower)
	{
		iot_set_low_power(skfd,pIFname);
		goto _exit;
	}
	if(bIsGetInfo)
	{
		setifflags(skfd, pIFname, IFF_UP);
		
		linkstatsus = iot_get_netlink_status(skfd, pIFname);
				
		printf("Net link status: %s\n", linkstatsus==1?"up":"down");

		if(iot_get_info(skfd, pIFname, &info)==0)
		{
			printf("IoT Device %s Info: \n", info.name);
			printf("Status: 0x%02x\n",	info.status&0xff);
			printf("Event Type: 0x%02x\n",	info.status>>8);

			if((info.status&0xff)==0x84)
				printf("IP Address: %d.%d.%d.%d\n",info.ipaddr&0xff, (info.ipaddr>>8)&0xff,(info.ipaddr>>16)&0xff, info.ipaddr>>24 );
			else 
				printf("IP invalid"); 
			printf("wifi profile info:\n");
			printf("SSID: %s\n",info.wifisetting.ssid);
			printf("Password: %s\n",info.wifisetting.psk);
			printf("Auth: %d\n",info.wifisetting.auth);
			printf("Encrypt: %d\n",info.wifisetting.encrypt);

			if(linkstatsus==1)
				iot_set_ifaddr(skfd, pIFname, info.ipaddr);			
		}
	}

	if(strlen(wifi.ssid))
	{
		setifflags(skfd, pIFname, -IFF_UP);

		if(ret = iot_set_wifi_setting(skfd, pIFname, &wifi))
			printf("Set Wifi faield!(%d)\n",ret);      
		else
			printf("Set Wifi Successed!\n");      

		
		if(ret = iot_get_wifi_setting(skfd, pIFname, &wifi))
			printf("Get Wifi faield!(%d)\n",ret);      
		else
			printf("Get Wifi Successed!\n");
	}

_exit:
	close(skfd);
    
	return 0;
}



