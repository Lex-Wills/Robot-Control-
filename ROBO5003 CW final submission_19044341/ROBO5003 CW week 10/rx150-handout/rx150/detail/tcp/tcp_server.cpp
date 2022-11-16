//
// Created by matthias on 11/03/18.
//

#include "tcp_server.h"

#include <boost/bind.hpp>
#include <thread>

using boost::asio::ip::tcp;
using namespace std;

tcp_server::tcp_server(boost::asio::io_context& io_context, short port, std::shared_ptr<RX150RobotInterface> robotP)
: io_context_(io_context), acceptor_(io_context, tcp::endpoint(tcp::v4(), port)), robot(robotP)
{
start_accept();
}

void tcp_server::start_accept()
{
    //tcp_connection::pointer new_connection =
    //        tcp_connection::create(acceptor_.get_io_service(), robot);
    tcp_connection::pointer new_connection =
            tcp_connection::create(io_context_, robot);

    acceptor_.async_accept(new_connection->socket(),
                           boost::bind(&tcp_server::handle_accept, this, new_connection,
                                       boost::asio::placeholders::error));
                                       
    //chat_session_ptr new_session(new chat_session(io_context_, room_));
    //acceptor_.async_accept(new_session->socket(),
    //    boost::bind(&chat_server::handle_accept, this, new_session,
    //      boost::asio::placeholders::error));
}

void tcp_server::handle_accept(tcp_connection::pointer new_connection,
                   const boost::system::error_code& error){
    if (!error)
    {
        thread newThread(
                [new_connection](){new_connection->start();}
        );
        newThread.detach();
        //new_connection->start();
    }

    start_accept();
}
