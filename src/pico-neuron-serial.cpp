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
#include <pthread.h>
#include <pwd.h>
#include <sched.h>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
using namespace std;
using namespace std::chrono;
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
steady_clock::time_point start_time;
bool real_measure = false;

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
    steady_clock::time_point endtime = steady_clock::now();
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

    file << std::fixed << std::setprecision(8);
    file << "x" << separator << "time\n";
    for (size_t i = 0; i < x.size(); ++i) {
      file << x[i] << separator << t[i] << "\n";
    }

    file.close();
    cout << "Data written to: " << path << endl;
    cout << "Program runned for: "
         << duration_cast<seconds>(endtime - start_time).count() << " seconds"
         << endl;
  }
};

// Global variables
DataWriter *data_writer = nullptr;
vector<double> x;
vector<double> t;
int serial_fd;
float time_counter = 0;
float incrr = 0.005;

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
  case 1000000:
    return B1000000;
  case 1152000:
    return B1152000;
  case 1500000:
    return B1500000;
  case 2000000:
    return B2000000;
  case 3000000:
    return B3000000;
  default:
    cerr << "Unsupported baud rate. Using 9600." << endl;
    return B9600;
  }
}

void *serial_read_thread(void *arg) {
  const size_t BUF_SIZE = 1024;
  char buf[BUF_SIZE];
  string partial_line;
  steady_clock::time_point read_time;
  bool clock_activated = false;

  while (true) {
    int n = read(serial_fd, buf, sizeof(buf) - 1);
    if (n > 0) {
      if (!clock_activated && real_measure) {
        read_time = steady_clock::now();
        clock_activated = true;
      }
      buf[n] = '\0';
      partial_line += string(buf); // Append new data

      size_t pos;
      while ((pos = partial_line.find('\n')) != string::npos) {
        string line = partial_line.substr(0, pos);
        partial_line.erase(0, pos + 1); // Remove processed line

        if (line.empty())
          continue;

        try {
          float value = stof(line);
          x.push_back(value);
          if (real_measure) {
            t.push_back(
                duration_cast<microseconds>(steady_clock::now() - read_time)
                    .count() /
                1000000.0);
          } else {
            t.push_back(time_counter);
          }
          time_counter += incrr;
        } catch (const invalid_argument &e) {
          if (line == "END") {
            cout << "\nSaving data to "
                 << data_writer->directory + data_writer->filename << endl;
            data_writer->write(x, t);
            close(serial_fd);
            exit(0);
          } else {
            cerr << "Conversion error: [" << line << "] " << e.what() << endl;
          }
        }
      }
    }
  }
  return nullptr;
}

int main(int argc, char *argv[]) {
  string directory = "data/";
  string filename = "hindmarsh-rose.csv";
  string separator = " ";
  string serial_name = "/dev/ttyUSB0";
  int serial_rate = 1000000;

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
    else if ((arg == "-sr" || arg == "--serial-rate") && i + 1 < argc)
      serial_rate = stoi(argv[++i]);
    else if ((arg == "-rm" || arg == "--real-measure") && i < argc)
      real_measure = true;
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

  configure_serial(serial_fd, get_baudrate(serial_rate));

  start_time = steady_clock::now();

  char buf[50];
  string partial_line;

  pthread_t thread;
  pthread_attr_t attr;
  int ret;

  struct sched_param param;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    printf("mlockall failed: %m\n");
    exit(-2);
  }
  pthread_attr_init(&attr);
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
  if (ret) {
    printf("pthread setstacksize failed\n");
    exit(-2);
  }

  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
    printf("pthread setschedpolicy failed\n");
    exit(-2);
  }
  param.sched_priority = 80;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
    printf("pthread setschedparam failed\n");
    exit(-2);
  }
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
    printf("pthread setinheritsched failed\n");
    exit(-2);
  }
  ret = pthread_create(&thread, &attr, serial_read_thread, NULL);
  if (ret) {
    cerr << "Failed to create real-time thread: " << strerror(ret) << endl;
  }
  ret = pthread_join(thread, NULL);
  if (ret)
    printf("join pthread failed: %m\n");
  pthread_attr_destroy(&attr);

  return 0;
}
