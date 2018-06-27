#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

auto eqto(uint8_t v) { return [v](uint8_t k) { return k == v; }; }

constexpr uint16_t UploadPort = 65512;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: " << argv[0] << " <ip-address> <bin-file>" << endl;
        return -1;
    }
    
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0)
    {
        cerr << "Error: could not create socket (error " << s << ")!" << endl;
        return -1;
    }
    
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(UploadPort);
    
    timeval timeout;
    timeout.tv_sec = 1200;
    timeout.tv_usec = 0;
    
#define GUARD(expr, str) do { auto e = (expr); if (e < 0) { cout << (str) << endl; return -1; } } while (0)

    GUARD(setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)), "Error: could not set timeout!");
    GUARD(inet_pton(AF_INET, argv[1], &address.sin_addr)-1, "Error: invalid address given!");
    GUARD(connect(s, (sockaddr*)&address, sizeof(address)), "Error: could not connect to socket!");

    cout << "Connected to remote upload server!" << endl;
    
    ifstream file(argv[2], ios::binary);
    file.seekg(0, ios::end);
    size_t size = file.tellg();
    file.seekg(0, ios::beg);
    
    cout << "File " << argv[2] << " has " << size << " bytes" << endl;
    if (size >= 62*1024)
    {
        cerr << "Error: file too big to upload!" << endl;
        return -1;
    }
    
    size_t pages = (size+1023)/1024;
    cout << "There are " << pages << " pages to upload." << endl;
    
    cout << "Sending update request..." << endl;
    
    GUARD(send(s, "update", 6, 0), "Error: could not send the update request!");
    
    // Wait for the confirmation bytes
    int8_t result;
    GUARD(recv(s, &result, 1, 0), "Error: could not receive the confirmation bytes!");
    
    if (result != 72)
    {
        cerr << "Error: upload confirmation incorrect!"<< endl;
        return -1;
    }
    
    cout << "Sending number of pages (" << pages << ")..." << endl;
    uint8_t pgnum = pages;
    GUARD(send(s, &pgnum, 1, 0), "Error: could not send page number!");
    GUARD(recv(s, &result, 1, 0), "Error: could not receive next command!");
    
    if (result != 0)
    {
        cerr << "Error: incorrect byte received!" << endl;
        return -1;
    }
    
    cout << "Ready to send pages!" << endl;
    
    for (size_t i = 0; i < pages; i++)
    {
        size_t read_num = min<size_t>(size - 1024*i, 1024);
        uint8_t cur_page[1024];
        file.read((char*)cur_page, read_num);
        std::fill(cur_page+read_num, cur_page+1024, 0);
        
        cout << "Sending page " << i << "..." << endl;
        GUARD(send(s, cur_page, 1024, 0), "Error: could not send page!");
        GUARD(recv(s, &result, 1, 0), "Error: could not receive command!");

        if (result != i+1)
        {
            cerr << "Error: not recognized success byte!"<< endl;
            return -1;
        }
    }
    
    cout << "Program uploading complete." << endl;
}
