//
// Created by matthias on 11/03/18.
//

#ifndef AX18A_SERVERCLIENT_TCP_SERVER_H
#define AX18A_SERVERCLIENT_TCP_SERVER_H

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

#include "tcp_connection.h"
#include "../../rx150/RX150RobotInterface.h"

class tcp_server
{
public:
    tcp_server(boost::asio::io_context& io_context, short port, std::shared_ptr<RX150RobotInterface> robotP);

private:
    void start_accept();

    void handle_accept(tcp_connection::pointer new_connection,
                       const boost::system::error_code& error);

    boost::asio::io_context& io_context_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::shared_ptr<RX150RobotInterface> robot;
};


#endif //AX18A_SERVERCLIENT_TCP_SERVER_H
