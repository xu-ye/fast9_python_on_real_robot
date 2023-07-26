#coding=GBK"
import time
import datetime
import platform
import struct
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver


def readConfig(device):
    """
    ��ȡ������Ϣʾ��    Example of reading configuration information
    :param device: �豸ģ�� Device model
    :return:
    """
    tVals = device.readReg(0x02,3)  #��ȡ�������ݡ��ش����ʡ�ͨѶ����   Read data content, return rate, communication rate
    if (len(tVals)>0):
        print("���ؽ����" + str(tVals))
    else:
        print("�޷���")
    tVals = device.readReg(0x23,2)  #��ȡ��װ�����㷨  Read the installation direction and algorithm
    if (len(tVals)>0):
        print("���ؽ����" + str(tVals))
    else:
        print("�޷���")

def setConfig(device):
    """
    ����������Ϣʾ��    Example setting configuration information
    :param device: �豸ģ�� Device model
    :return:
    """
    device.unlock()                # ���� unlock
    time.sleep(0.1)                # ����100����    Sleep 100ms
    device.writeReg(0x03, 6)       # ���ûش�����Ϊ10HZ    Set the transmission back rate to 10HZ
    time.sleep(0.1)                # ����100����    Sleep 100ms
    device.writeReg(0x23, 0)       # ���ð�װ����:ˮƽ����ֱ   Set the installation direction: horizontal and vertical
    time.sleep(0.1)                # ����100����    Sleep 100ms
    device.writeReg(0x24, 0)       # ���ð�װ����:���ᡢ����   Set the installation direction: nine axis, six axis
    time.sleep(0.1)                # ����100����    Sleep 100ms
    device.save()                  # ���� Save

def AccelerationCalibration(device):
    """
    �Ӽ�У׼    Acceleration calibration
    :param device: �豸ģ�� Device model
    :return:
    """
    device.AccelerationCalibration()                 # Acceleration calibration
    print("�Ӽ�У׼����")

def FiledCalibration(device):
    """
    �ų�У׼    Magnetic field calibration
    :param device: �豸ģ�� Device model
    :return:
    """
    device.BeginFiledCalibration()                   # ��ʼ�ų�У׼   Starting field calibration
    if input("��ֱ���XYZ������ת��һȦ������תȦ��ɺ󣬽���У׼��Y/N)��").lower()=="y":
        device.EndFiledCalibration()                 # �����ų�У׼   End field calibration
        print("�����ų�У׼")


def startRecord():
    """
    ��ʼ��¼����  Start recording data
    :return:
    """
    global _writeF
    global _IsWriteF
    _writeF = open(str(datetime.datetime.now().strftime('%Y%m%d%H%M%S')) + ".txt", "w")    #�½�һ���ļ�
    _IsWriteF = True                                                                        #���д���ʶ
    Tempstr = "Chiptime"
    Tempstr +=  "\tax(g)\tay(g)\taz(g)"
    Tempstr += "\twx(deg/s)\twy(deg/s)\twz(deg/s)"
    Tempstr += "\tAngleX(deg)\tAngleY(deg)\tAngleZ(deg)"
    Tempstr += "\tT(��)"
    Tempstr += "\tmagx\tmagy\tmagz"
    Tempstr += "\tlon\tlat"
    Tempstr += "\tYaw\tSpeed"
    Tempstr += "\tq1\tq2\tq3\tq4"
    Tempstr += "\r\n"
    _writeF.write(Tempstr)
    print("��ʼ��¼����")

def endRecord():
    """
    ������¼����  End record data
    :return:
    """
    global _writeF
    global _IsWriteF
    _IsWriteF = False             # ��ǲ���д���ʶ    Tag cannot write the identity
    _writeF.close()               #�ر��ļ� Close file
    print("������¼����")  
