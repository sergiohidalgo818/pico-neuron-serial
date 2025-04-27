// receiver.cpp
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <grp.h>
#include <iomanip>
#include <iostream>
#include <pwd.h>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

using namespace std;
using namespace std::chrono;

class DataWriter {
public:
  string directory;
  string filename;
  string separator;

  DataWriter(const string &directory, const string &filename,
             const string &separator)
      : directory(directory), filename(filename), separator(separator) {}

  void write(const vector<double> &x, const vector<double> &t) {
    struct stat info;
    if (stat(directory.c_str(), &info) != 0) {
      cout << "Directory does not exist. Creating: " << directory << endl;
      mkdir(directory.c_str(), 0755);
    } else if (!(info.st_mode & S_IFDIR)) {
      cerr << directory << " is not a directory!" << endl;
      return;
    }

    string path = directory + filename;
    ofstream file(path);
    if (!file.is_open()) {
      cerr << "Error opening file: " << path << endl;
      return;
    }

    file << std::fixed;
    file << std::setprecision(8); // up to 8 decimal places

    file << "x" << separator << "time\n";
    for (size_t i = 0; i < x.size(); ++i) {
      file << x[i] << separator << t[i] << "\n";
    }
    file.close();
  }
};

// Global variables
DataWriter *data_writer = nullptr;
vector<double> x;
vector<double> t;
int serial_fd;
steady_clock::time_point start_time;

void signal_handler(int signum) {
  cout << "\nSaving data to " << data_writer->directory + data_writer->filename
       << endl;
  data_writer->write(x, t);
  close(serial_fd);
  exit(0);
}

void check_permissions() {
  uid_t uid = getuid();
  if (uid == 0) {
    cerr << "Do not run this script as root. Please run it as a normal user."
         << endl;
    exit(1);
  }

  struct passwd *pw = getpwuid(uid);
  if (!pw) {
    cerr << "Failed to get user information." << endl;
    exit(1);
  }
  string username(pw->pw_name);

  struct group *grp_info = getgrnam("dialout");
  if (!grp_info) {
    cerr << "Group 'dialout' not found on system." << endl;
    exit(1);
  }

  bool found = false;
  for (char **members = grp_info->gr_mem; *members != nullptr; ++members) {
    if (username == *members) {
      found = true;
      break;
    }
  }

  if (!found) {
    cerr << "User '" << username << "' is not in the 'dialout' group." << endl;
    cerr << "To fix this, run: sudo usermod -aG dialout $USER" << endl;
    cerr << "Then log out and log back in." << endl;
    exit(1);
  }
}

void configure_serial(int fd, int baudrate) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd, &tty) != 0) {
    cerr << "Error from tcgetattr" << endl;
    exit(1);
  }

  cfsetospeed(&tty, baudrate);
  cfsetispeed(&tty, baudrate);

  tty.c_cflag |= (CLOCAL | CREAD); // enable receiver, local mode
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;      // 8-bit characters
  tty.c_cflag &= ~PARENB;  // no parity bit
  tty.c_cflag &= ~CSTOPB;  // only need 1 stop bit
  tty.c_cflag &= ~CRTSCTS; // no hardware flowcontrol

  tty.c_lflag = 0;      // no signaling chars, no echo
  tty.c_oflag = 0;      // no remapping, no delays
  tty.c_cc[VMIN] = 0;   // read doesn't block
  tty.c_cc[VTIME] = 10; // 1 second read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    cerr << "Error from tcsetattr" << endl;
    exit(1);
  }
}

speed_t get_baudrate(int baudrate) {
  switch (baudrate) {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  default:
    cerr << "Unsupported baud rate. Using 9600." << endl;
    return B9600;
  }
}

int main(int argc, char *argv[]) {
  string directory = "data/";
  string filename = "hindmarsh-rose.csv";
  string separator = " ";
  string serial_name = "/dev/ttyUSB0";
  int serial_id = 9600;

  // Simple argument parsing
  for (int i = 1; i < argc; ++i) {
    string arg = argv[i];
    if ((arg == "-d" || arg == "--directory") && i + 1 < argc)
      directory = argv[++i];
    else if ((arg == "-f" || arg == "--filename") && i + 1 < argc)
      filename = argv[++i];
    else if ((arg == "-s" || arg == "--separator") && i + 1 < argc)
      separator = argv[++i];
    else if ((arg == "-sn" || arg == "--serial-name") && i + 1 < argc)
      serial_name = argv[++i];
    else if ((arg == "-si" || arg == "--serial-id") && i + 1 < argc)
      serial_id = stoi(argv[++i]);
  }

  check_permissions();

  data_writer = new DataWriter(directory, filename, separator);

  signal(SIGINT, signal_handler);

  cout << "Press Ctrl+C to save the file with the data" << endl;

  serial_fd = open(serial_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd < 0) {
    cerr << "Error opening serial port " << serial_name << endl;
    return 1;
  }

  configure_serial(serial_fd, get_baudrate(serial_id));

  start_time = steady_clock::now();

  char buf[50];
  while (true) {
    int n = read(serial_fd, buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      string line(buf);
      size_t pos = line.find('\n');
      if (pos != string::npos) {
        line = line.substr(0, pos);
      }
      if (line.empty())
        continue; // Skip empty lines

      try {
        float value = stof(line);
        auto now = steady_clock::now();
        double elapsed =
            duration_cast<nanoseconds>(now - start_time).count() / 1e9;
        x.push_back(value);
        t.push_back(elapsed);
      } catch (const invalid_argument &e) {
        cerr << "Conversion error: " << e.what() << endl;
      }
    }
  }

  return 0;
}
