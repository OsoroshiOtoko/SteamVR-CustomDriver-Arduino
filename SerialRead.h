#include <windows.h>
#include <iostream>
#include <fstream>
#include <tchar.h>
#include <string.h>
#include <string>
#include <thread>


using namespace std;

HANDLE hSerial;

string str_data; //строка (название число)
string value_name; //int Value(); 

bool DeviceConnected = false;






int Value(string fname)
{
    int len;

    string s_value;
    string temp_value;
    string temp_name = "";
    int lp = 999;
    int i = 0;

    len = str_data.length() - 2;

    temp_name = "";
    for (i = 0; i < len; i++) {
        if (str_data[i] == ' ') {
            value_name = temp_name;
            if (value_name == fname) {
                do {
                    i++;
                    temp_value += str_data[i];
                } while (i < len);
                s_value = temp_value;
                temp_value = "";
                lp = stoi(s_value);
            }
        }
        else temp_name += str_data[i];
    }
    return lp;
}




int SerialRead(string name) {
    DWORD iSize;
    char sReceivedChar = '\0';
    string str_temp = "";
    int ax = 999;

    //while (HMDConnected) {
    //ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);  // получаем 1 байт
    //cout << sReceivedChar << endl;
    while (value_name != name) {
        while (sReceivedChar != '*')
        {
            ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);

        }
        if (sReceivedChar == '*') {
            while (sReceivedChar != '\n')
            {
                ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
                str_temp += sReceivedChar;
            }

            str_data = str_temp;
            str_temp = "";
            if (Value(name) != 999) ax = Value(name);
        }
    }
    value_name = "";
    return ax;
}



void SerialBegin(LPCTSTR sPortName) {
    if (DeviceConnected == false) {
        //LPCTSTR sPortName = L"COM5";
        hSerial = ::CreateFile(sPortName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

        if (hSerial == INVALID_HANDLE_VALUE)
        {
            if (GetLastError() == ERROR_FILE_NOT_FOUND)
            {
                cout << "serial port does not exist.\n";
            }
            cout << "some other error occurred.\n";
        }

        /*COMMTIMEOUTS CommTimeOuts = {0xFFFFFFFF,0,0,0,1500};

        if (!SetCommTimeouts(hSerial, &CommTimeOuts))
        {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            cout << "Ошибка при записи таймаутов порта";
        } */

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams))
        {
            cout << "getting state error\n";
        }
        dcbSerialParams.BaudRate = CBR_9600;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        if (!SetCommState(hSerial, &dcbSerialParams))
        {
            cout << "error setting serial port state\n";
        }

        DeviceConnected = true;
    }
}
