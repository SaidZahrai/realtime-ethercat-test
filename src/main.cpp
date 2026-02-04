#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::string iface = "eth0";
    for (int i=1; i<argc; ++i) {
        std::string a = argv[i];
        if (a == "--iface" && i+1 < argc) iface = argv[++i];
    }
    std::cout << "realtime-ethercat (skeleton) using iface=" << iface << "\n";
    return 0;
}
