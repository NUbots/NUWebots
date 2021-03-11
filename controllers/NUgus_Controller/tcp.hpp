#ifndef CONTROLLERS_NUGUS_CONTROLLER_TCP
#define CONTROLLERS_NUGUS_CONTROLLER_TCP

#include <sys/socket.h>
#include <vector>

namespace tcp {
    /// @brief An established tcp connection
    class Connection{
    private:
        ///@brief the socket to read from and write to
        int socket_fd;
    public:
        Connection();
        /// @brief Reads a number of bytes from the socket
        std::vector<char> read(int);
        /// @brief Writes an array of bytes to the server
        void write(std::vector<char>);
        /// @brief Closes the connection
        void close();
        /// @brief Closes the connection because RAII is cool
        ~Connection();
    };

    /// @brief Listens for tcp connections
    class Server{
    private:
        int server_fd;
    public:
        std::vector<Connection> connections();
        void close();
    };

    /// @brief connects to a tcp server
    Connection connect(sockaddr&&);

    /// @brief creates a tcp server
    Server listen(sockaddr&&);
}

#endif //CONTROLLERS_NUGUS_CONTROLLER_TCP
