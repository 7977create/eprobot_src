import os
import re
 
# Return CPU temperature as a character string                                      
def getCPUtemperature():
    res = os.popen('cat /sys/class/thermal/thermal_zone0/temp').readline()
    return(str(float(res.replace("temp=","").replace("'C\n",""))/1000.0))
 
# Return RAM information (unit=kb) in a list                                       
# Index 0: total RAM                                                               
# Index 1: used RAM                                                                 
# Index 2: free RAM                                                                 
def getRAMinfo():
    p = os.popen('free')
    i = 0
    while 1:
        i = i + 1
        line = p.readline()
        if i==2:
            return(line.split()[1:4])
 
# Return % of CPU used by user as a character string                                
def getCPUuse():
    return(str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print $2}'").readline().strip()))
 
# Return information about disk space as a list (unit included)                     
# Index 0: total disk space                                                         
# Index 1: used disk space                                                         
# Index 2: remaining disk space                                                     
# Index 3: percentage of disk used                                                  
def getDiskSpace():
    p = os.popen("df -h /")
    i = 0
    while 1:
        i = i +1
        line = p.readline()
        if i==2:
            return(line.split()[1:5])
 
 
# CPU informatiom
CPU_temp = getCPUtemperature()
CPU_usage = getCPUuse()
 
# RAM information
# Output is in kb, here I convert it in Mb for readability
RAM_stats = getRAMinfo()
RAM_total = round(int(RAM_stats[0]) / 1000,1)
RAM_used = round(int(RAM_stats[1]) / 1000,1)
RAM_free = round(int(RAM_stats[2]) / 1000,1)
 
# Disk information
DISK_stats = getDiskSpace()
DISK_total = DISK_stats[0]
DISK_used = DISK_stats[1]
DISK_perc = DISK_stats[3]

# use os.popen() exec ifconfig, result is file object, put into cmd_file to save
cmd_file = os.popen('ifconfig wlan0')
# use file object's read() to get cmd_file 
cmd_result = cmd_file.read()
# mode of match IP
pattern = re.compile(r'(inet )(\d{1,3}.\d{1,3}.\d{1,3}.\d{1,3})')
# use re moudle to mathc findall
ip_wlan0 = re.findall(pattern, cmd_result)

# use os.popen() exec ifconfig, result is file object, put into cmd_file to save
cmd_file = os.popen('ifconfig eth0')
# use file object's read() to get cmd_file 
cmd_result = cmd_file.read()
# use re moudle to mathc findall
ip_eth0 = re.findall(pattern, cmd_result)

cmd_file = os.popen('iwconfig wlan0')
cmd_result = cmd_file.read()
pattern = re.compile(r'(Mode:)(\w{1,7})')
wlan0_mode = re.findall(pattern, cmd_result)
 
if __name__ == '__main__':
    print('')
    print('CPU Temperature = '+CPU_temp)
    print('CPU Use = '+CPU_usage)
    print('')
    print('RAM Total = '+str(RAM_total)+' MB')
    print('RAM Used = '+str(RAM_used)+' MB')
    print('RAM Free = '+str(RAM_free)+' MB')
    print('RAM Used Percentage = '+str(RAM_used*100/RAM_total)+'%')
    print('')  
    print('DISK Total Space = '+str(DISK_total)+'B')
    print('DISK Used Space = '+str(DISK_used)+'B')
    print('DISK Used Percentage = '+str(DISK_perc))
    if len(ip_wlan0) == 0:
        ip_wlan0_list=['0','0','0','0']
        print ip_wlan0_list
        print('wlan0 is not connected') 
    else:
        ip_wlan0_list=ip_wlan0[0][1].split('.')
        print('waln0 IP:'+(ip_wlan0_list[0])+'.'\
            +(ip_wlan0_list[1])+'.'+(ip_wlan0_list[2])+'.'
            +(ip_wlan0_list[3]))    
    if len(ip_eth0) == 0:
        print('eth0 is not connected') 
    else:
        ip_eth0_list=ip_eth0[0][1].split('.')
        print('eth0 IP:'+(ip_eth0_list[0])+'.'\
            +(ip_eth0_list[1])+'.'+(ip_eth0_list[2])+'.'
            +(ip_eth0_list[3])) 
    list_all = ip_wlan0_list + ip_eth0_list
    list_all.append(wlan0_mode[0][1])
    print list_all
