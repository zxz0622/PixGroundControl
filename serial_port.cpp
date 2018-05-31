#include "serial_port.h"

// ------------------------------------------------------------------------------
//   init serial
// ------------------------------------------------------------------------------
void Serial_Port::initialize_defaults()
{
    // Initialize attributes
    status = false;
    debug = false;
    //status = SERIAL_PORT_CLOSED;

    uart_name = (char*)"com4";
    baudrate  = 57600;
    datebits = 8;
    stopbit = 1;
    parity = 0;
    /*// Start mutex
    int result = pthread_mutex_init(&lock, NULL);
    if ( result != 0 )
    {
        printf("\n mutex init failed\n");
        throw 1;
    }*/
}

Serial_Port::Serial_Port()// : QObject(parent)
{
    initialize_defaults();
}
Serial_Port::Serial_Port(const char *uart_name_, int baudrate_)// : QObject(parent)
{
    initialize_defaults();
    uart_name = uart_name_;
    baudrate = baudrate_;
}
Serial_Port::~Serial_Port(){
    close_serial();
}

// ------------------------------------------------------------------------------
//   open serial
// ------------------------------------------------------------------------------
void Serial_Port::open_serial(){

    if (!status)
    {

        mSerialPort = new QSerialPort();

    }
    printf("OPEN PORT\n");
    //打开串口
    mSerialPort->setPortName(uart_name);//设置串口
    if (mSerialPort->open(QIODevice::ReadWrite))//打开串口
    {
        mSerialPort->setBaudRate(baudrate);//设置波特率

        switch(datebits)
        {
            case 8:mSerialPort->setDataBits(QSerialPort::Data8);break;
            case 7:mSerialPort->setDataBits(QSerialPort::Data7);break;
            case 6:mSerialPort->setDataBits(QSerialPort::Data6);break;
            case 5:mSerialPort->setDataBits(QSerialPort::Data5);break;
        }
        mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

        switch(stopbit)
        {
            case 0:mSerialPort->setStopBits(QSerialPort::OneStop);break;
            case 1:mSerialPort->setStopBits(QSerialPort::OneAndHalfStop);break;
            case 2:mSerialPort->setStopBits(QSerialPort::TwoStop);break;

        }
        switch(parity)
        {
            case 0:mSerialPort->setParity(QSerialPort::NoParity);break;
            case 1:mSerialPort->setParity(QSerialPort::EvenParity);break;
            case 2:mSerialPort->setParity(QSerialPort::OddParity);break;
        }
        // 返回打开结果
        printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
        mSerialPort->clear();
        //connect(mSerialPort,SIGNAL(readyRead()),this,SLOT(read()));

        lastStatus.packet_rx_drop_count = 0;
        status = true;
        //emit finished(true);
    }
    else
    {
        printf("failure, could not open port.\n");
        //emit finished(false);
    }


}

// ------------------------------------------------------------------------------
//   close Serial
// ------------------------------------------------------------------------------
void Serial_Port::close_serial(){
    if (mSerialPort)
    {
        mSerialPort->close();
        delete mSerialPort;
        mSerialPort = NULL;
        status = false;
    }
}

// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
void Serial_Port::read()
{

    QByteArray data = mSerialPort->readAll();//读取串口上所有数据

    /*for(int i = 0;i!=data.size();i++){
        data_received.enqueue((uint8_t)data[i]);
    }*/

    data_received.append(data);
    //printf("readall completed\n");

    //sleep
    Sleep(1); //如果程序崩溃 增大延时
}

int Serial_Port::readbyte(uint8_t &cp){

    if(data_received.size()>500) {
        //printf("clear\n");
        data_received.clear();
    }
    //method dont use readyread connect
    if(data_received.isEmpty()){
        while(!mSerialPort->waitForReadyRead(50));
        data_received.append(mSerialPort->readAll());
    }
    /*while(data_received.isEmpty()){
        Sleep(1);
    }*/

    cp = data_received[0];
    data_received.remove(0,1);
//printf("data num = %d", data_received.size());
    return 1;

}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int Serial_Port::write_port(char *buf, unsigned len)
{
/*
    // Lock
    pthread_mutex_lock(&lock);

    // Write packet via serial link
    const int bytesWritten = static_cast<int>(write(fd, buf, len));

    // Wait until all data has been written
    tcdrain(fd);

    // Unlock
    pthread_mutex_unlock(&lock);


    return bytesWritten;*/

    int bytesWritten = (int)mSerialPort->write(buf, len);//发送串口数据
    return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Read from Serial & package
// ------------------------------------------------------------------------------
int
Serial_Port::
read_message(mavlink_message_t &message)
{
    uint8_t          cp;
    mavlink_status_t status;
    uint8_t          msgReceived = false;

    // --------------------------------------------------------------------------
    //   READ FROM PORT
    // --------------------------------------------------------------------------

    // this function locks the port during read

    int result = readbyte(cp);

    // --------------------------------------------------------------------------
    //   PARSE MESSAGE
    // --------------------------------------------------------------------------
    if (result > 0)
    {
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        // check for dropped packets
        if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
        {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
            unsigned char v=cp;
            printf("%02x ", v);
        }
        lastStatus = status;
    }

    // Couldn't read from port
    else
    {
        printf("ERROR: Could not read from uart %s\n",uart_name);
    }

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if(msgReceived && debug)
    {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

        printf("Received serial data: ");
        unsigned int i;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN)
        {
            printf( "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
        }

        // print out the buffer
        else
        {
            for (i=0; i<messageLength; i++)
            {
                unsigned char v=buffer[i];
                printf("%02x ", v);
            }
            printf("\n");
        }
    }

    // Done!
    return msgReceived;
}


// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
Serial_Port::
write_message(const mavlink_message_t &message)
{
    char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = write_port(buf,len);

    return bytesWritten;
}

