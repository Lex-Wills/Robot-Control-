//
// Created by matthias on 11/03/18.
//

#ifndef RX150_SERVERCLIENT_TCP_CONNECTION_H
#define RX150_SERVERCLIENT_TCP_CONNECTION_H

#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include "../../rx150/RX150RobotInterface.h"

class tcp_connection : public boost::enable_shared_from_this<tcp_connection>{
public:
    typedef boost::shared_ptr<tcp_connection> pointer;

    static pointer create(boost::asio::io_context& io_context, std::shared_ptr<RX150RobotInterface> robotP);

    boost::asio::ip::tcp::socket& socket();

    void start();

private:
    void handleClient();
    tcp_connection(boost::asio::io_context& io_context, std::shared_ptr<RX150RobotInterface> robotP);
private:
    boost::asio::ip::tcp::socket socket_;
    std::shared_ptr<RX150RobotInterface> robot;
};


#endif //AX18A_SERVERCLIENT_TCP_CONNECTION_H
