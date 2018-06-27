#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <serial/serial.h>

using namespace std;
using namespace serial;

auto eqto(uint8_t v) { return [v](uint8_t k) { return k == v; }; }

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: " << argv[0] << " <out-port> <bin-file>" << endl;
        return -1;
    }
    
    Serial portSerial(argv[1], 57600);
    if (!portSerial.isOpen())
    {
        cerr << "Error: could not open serial port!" << endl;
        return -1;
    }
    
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
    
    cout << "Writing update request..." << endl;
    portSerial.write(std::vector<uint8_t>(32, 0x77));
    
    // Wait for the confirmation bytes
    std::vector<uint8_t> sentinel;
    while (portSerial.available() < 9);
    portSerial.read(sentinel, 9);
    
    if (!std::all_of(sentinel.begin(), sentinel.end(), eqto(0x66)))
    {
        cerr << "Error: upload confirmation incorrect!"<< endl;
        return -1;
    }
    
    cout << "Sending number of pages (" << pages << ")..." << endl;
    std::vector<uint8_t> pageCtl(9, 0x97);
    pageCtl.push_back(pages);
    portSerial.write(pageCtl);
    
    while (portSerial.available() < 9);
    sentinel.clear();
    portSerial.read(sentinel, 9);
    
    if (!std::all_of(sentinel.begin(), sentinel.end(), eqto(0x63)))
    {
        cerr << "Error: unexpected byte sequence for number of pages!"<< endl;
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
        portSerial.write(std::vector<uint8_t>(12, 0x33 + 3*i));
        portSerial.write(cur_page, 1024);
        
        sentinel.clear();
        while (portSerial.available() < 9);
        portSerial.read(sentinel, 9);
        
        if (!std::all_of(sentinel.begin(), sentinel.end(), eqto(0x97 - 7*i)))
        {
            cerr << "Error: not recognized success byte!"<< endl;
            return -1;
        }
    }
    
    cout << "Program uploading complete." << endl;
}
