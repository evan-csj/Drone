$port=new-Object System.IO.Ports.SerialPort COM3,9600,None,8,one
$port.RtsEnable=1
$port.DtrEnable=1
$port.open()

for($i=0;;$i++){
	$port.ReadLine()
}