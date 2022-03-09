// Definition of the Socket class

#ifndef ClpeSocket_class
#define ClpeSocket_class
 
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <iostream>

#define SOCKET_CLIENT
  
const int MAXCONNECTIONS = 5;

using namespace std;

class ClpeSocket
{
public:
	ClpeSocket();
	virtual ~ClpeSocket();

	// Server initialization
	bool create(int mcu_id);
	bool bind(const char *host ,const int port, int mcu_id);
	bool listen(int mcu_id) const;
	bool accept(ClpeSocket *new_socket, int mcu_id) const;

	// Client initialization
	bool connect(const string host, const int port, int mcu_id);

	// Data Transimission
	bool send(unsigned char *s, int mcu_id) const;
	bool recv(unsigned char *s, int mcu_id) const;

	void close(int mcu_id);

	void set_non_blocking(const bool, int mcu_id);

	bool is_valid_master() const { return m_sock_master != -1; }
    bool is_valid_slave() const { return m_sock_slave!= -1; }

private:
	int m_sock_master;
    int m_sock_slave;
	sockaddr_in m_addr;

};


#endif
