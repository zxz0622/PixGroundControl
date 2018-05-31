#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <QObject>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <cstdlib>
//#include <QQueue>
#include <windows.h>
#include <mavlink/common/mavlink.h>

class Serial_Port : public QObject
{
    Q_OBJECT
public:
    explicit Serial_Port();
    explicit Serial_Port(const char *uart_name_, int baudrate_);
    void initialize_defaults();
    ~Serial_Port();

    static bool initFlag;
    const char *uart_name;
    int baudrate;
    int datebits;
    int stopbit;
    int parity;
    bool  status;
    bool debug;

    int read_message(mavlink_message_t &message);
    int write_message(const mavlink_message_t &message);
    void open_serial();
    void close_serial();
    int readbyte(uint8_t &cp);

private:
    QSerialPort *mSerialPort;//声明serial对象
    QByteArray data_received;
    mavlink_status_t lastStatus;
    int write_port(char *buf, unsigned len);


public slots:
    //int send(char *buf,int len);


private slots:
    void read();

signals:
    //void finished(bool);

};

#endif // SERIAL_PORT_H
