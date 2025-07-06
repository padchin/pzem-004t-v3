package pzem

import (
	"time"

	"github.com/padchin/pzem-004t-v3/crc16"

	"github.com/go-errors/errors"
	"github.com/padchin/serial"
)

type Register uint16
type Command uint8

const (
	////VOLTAGE value 1LSB correspond to 0.1V
	//VOLTAGE Register = 0x0000
	////INTENSITY_LOW 1LSB correspond to 0.001A
	//INTENSITY_LOW Register = 0x0001
	////INTENSITY_HIGH 1LSB correspond to 0.001A
	//INTENSITY_HIGH Register = 0X0002
	////POWER_LOW 1LSB correspond to 0.1W
	//POWER_LOW Register = 0x0003
	////PowerHigh 1LSB correspond to 0.1W
	//PowerHigh Register = 0x0004
	////EnergyLow 1LSB correspond to 1Wh
	//EnergyLow Register = 0x0005
	////EnergyHigh 1LSB correspond to 1Wh
	//EnergyHigh Register = 0x0006
	////Frequency 1LSB correspond to 0.1Hz
	//Frequency Register = 0x0007
	////PowerFactor 1lSB correspond to 0.01
	//PowerFactor Register = 0x0008
	////Alarm status 0xFFFF  is alarm 0x0000 is not alarm
	//Alarm Register = 0x0009

	//MODBUS_RTU_ADDRESS the range is 0x0001-0x00F7
	MODBUS_RTU_ADDRESS Register = 0x0002
	////AlarmThreshold  1LSB correspond to 1W
	//AlarmThreshold Register = 0x0001
	//
	////ReadHoldingRegister command
	//ReadHoldingRegister Command = 0x03
	//READ_INPUT_REGISTER command
	READ_INPUT_REGISTER Command = 0x04
	//WRITE_SINGLE_REGISTER command
	WRITE_SINGLE_REGISTER Command = 0x06
	////Calibration command
	//Calibration Command = 0x41
	//RESET_ENERGY command
	RESET_ENERGY Command = 0x42

	DEVICE_UPDATE_TIME             = 1000
	DEVICE_DEFAULT_BAUD_RATE       = 9600
	DEVICE_DEFAULT_ADDRESS   uint8 = 0xF8
)

// Probe is PZEM interface
type Probe interface {
	Voltage() (float64, error)
	Power() (float64, error)
	Energy() (float64, error)
	Frequency() (float64, error)
	Intensity() (float64, error)
	PowerFactor() (float64, error)
	ResetEnergy() error
}

// Config PZEM initialization
type Config struct {
	Port         string
	Speed        int
	SlaveAddress uint8
	ReadTimeout  time.Duration
}

type pzem struct {
	port           *serial.Port
	addr           uint8
	voltage        float64
	current        float64
	power          float64
	energy         float64
	frequency      float64
	power_factor   float64
	alarms         uint16
	last_read_time time.Time
}

//func debug(buf []uint8) {
//	for _, v := range buf {
//		fmt.Printf("%.2x", v)
//	}
//	fmt.Println()
//}

// Setup initialize new PZEM device
func Setup(config Config) (Probe, error) {

	if config.Port == "" {
		return nil, errors.New("serial port must be set")
	}
	if config.Speed == 0 {
		config.Speed = DEVICE_DEFAULT_BAUD_RATE
	}

	if config.SlaveAddress == 0 {
		config.SlaveAddress = DEVICE_DEFAULT_ADDRESS
	}
	c := &serial.Config{Name: config.Port, Baud: config.Speed, ReadTimeout: config.ReadTimeout}
	s, err := serial.OpenPort(c)
	if err != nil {
		return nil, err
	}
	p := &pzem{port: s}
	p.InitDevice(config.SlaveAddress)
	return p, nil
}

func (p *pzem) SetSlaveAddress(addr uint8) error {
	if addr < 0x01 || addr > 0xF7 { // sanity check
		return errors.New("address provided is incorrect")
	}

	// Write the new address to the address register
	if err := p.SendCommand(WRITE_SINGLE_REGISTER, MODBUS_RTU_ADDRESS, uint16(addr), true); err != nil {
		return err
	}

	p.addr = addr // If successful, update the current slave address

	return nil
}

func (p *pzem) SendCommand(cmd Command, reg Register, val uint16, check bool) error {
	var aui_send_buffer = make([]uint8, 8)     // Send buffer
	var aui_response_buffer = make([]uint8, 8) // Response buffer (only used when check is true)

	aui_send_buffer[0] = p.addr     // Set slave address
	aui_send_buffer[1] = uint8(cmd) // Set command

	aui_send_buffer[2] = uint8(reg>>8) & 0xFF // Set high byte of register address
	aui_send_buffer[3] = uint8(reg) & 0xFF    // Set low byte =//=

	aui_send_buffer[4] = uint8(val>>8) & 0xFF // Set high byte of register value
	aui_send_buffer[5] = uint8(val) & 0xFF    // Set low byte =//=

	SetCRC(aui_send_buffer)

	n, err := p.port.Write(aui_send_buffer) // send frame
	if n < len(aui_send_buffer) || err != nil {
		if err != nil {
			return err
		}
		return errors.Errorf("try to send %d, but %d sent", len(aui_send_buffer), n)
	}

	time.Sleep(200 * time.Millisecond)

	if check {
		if err := p.Receive(aui_response_buffer); n <= 0 || err != nil { // if check enabled, read the response
			return err
		}

		// Check if response is same as send
		for i := 0; i < 8; i++ {
			if aui_send_buffer[i] != aui_response_buffer[i] {
				return errors.New("response should be the same than the request")
			}
		}
	}

	return nil
}

func (p *pzem) InitDevice(addr uint8) {
	if addr < 0x01 || addr > 0xF8 { // Sanity check of address
		p.addr = DEVICE_DEFAULT_ADDRESS
	}
	p.addr = addr

	if p.addr != DEVICE_DEFAULT_ADDRESS {
		_ = p.SetSlaveAddress(p.addr)
	}

}

func (p *pzem) UpdateValues() error {
	response := make([]uint8, 25)

	//If we read before the update time limit, do not update
	if p.last_read_time.Add(DEVICE_UPDATE_TIME * time.Millisecond).After(time.Now()) {
		return nil
	}

	// Read 10 registers starting at 0x00 (no check)
	if err := p.SendCommand(READ_INPUT_REGISTER, 0x00, 0x0A, false); err != nil {
		return err
	}

	if err := p.Receive(response); err != nil { // Something went wrong
		return err
	}

	// Update the current values
	p.voltage = float64(uint32(response[3])<<8| // Raw voltage in 0.1V
		uint32(response[4])) / 10.0

	p.current = float64(uint32(response[5])<<8| // Raw current in 0.001A
		uint32(response[6])|
		uint32(response[7])<<24|
		uint32(response[8])<<16) / 1000.0

	p.power = float64(uint32(response[9])<<8| // Raw power in 0.1W
		uint32(response[10])|
		uint32(response[11])<<24|
		uint32(response[12])<<16) / 10.0

	p.energy = float64(uint32(response[13])<<8| // Raw Energy in 1Wh
		uint32(response[14])|
		uint32(response[15])<<24|
		uint32(response[16])<<16) / 1000.0

	p.frequency = float64(uint32(response[17])<<8| // Raw Frequency in 0.1Hz
		uint32(response[18])) / 10.0

	p.power_factor = float64(uint32(response[19])<<8| // Raw pf in 0.01
		uint32(response[20])) / 100.0

	p.alarms = uint16(uint32(response[21])<<8 | // Raw alarm value
		uint32(response[22]))

	p.last_read_time = time.Now()

	return nil
}

func IsError(buf []uint8) error {
	if buf[1] == 0x84 {
		switch buf[2] {
		case 0x01:
			return errors.New("Illegal command")
		case 0x02:
			return errors.New("Illegal address")
		case 0x03:
			return errors.New("Illegal data")
		case 0x04:
			return errors.New("Slave error")
		default:
			return errors.New("Unknown error")
		}
	}
	return nil
}

func (p *pzem) Receive(resp []uint8) error {
	n, err := p.port.Read(resp)
	if err != nil {
		return err
	}

	if n != len(resp) {
		return errors.Errorf("should got %d, but %d received", len(resp), n)
	}

	if !CheckCRC(resp) {
		return errors.New("received CRC is not valid")
	}

	if err = IsError(resp); err != nil {
		return err
	}

	return nil
}

func CheckCRC(buf []uint8) bool {
	l := len(buf)
	if l <= 2 {
		return false
	}
	var crc = crc16.CRC(buf[:l-2])
	return (uint16(buf[l-2]) | uint16(buf[l-1])<<8) == crc
}

func SetCRC(buf []uint8) {
	l := len(buf)
	if l <= 2 {
		return
	}
	var crc = crc16.CRC(buf[:l-2])
	buf[l-2] = uint8(crc) & 0xFF
	buf[l-1] = uint8(crc>>8) & 0xFF

}

func (p *pzem) ResetEnergy() error {
	buffer := []uint8{0x00, uint8(RESET_ENERGY), 0x00, 0x00}
	reply := make([]uint8, 4)
	buffer[0] = p.addr

	SetCRC(buffer)

	_, _ = p.port.Write(buffer)

	time.Sleep(400 * time.Millisecond)

	err := p.Receive(reply)
	if err != nil {
		return err
	}

	return nil
}

func (p *pzem) Voltage() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.voltage, nil
}

func (p *pzem) Intensity() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.current, nil
}

func (p *pzem) Power() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.power, nil
}

func (p *pzem) Energy() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.energy, nil
}

func (p *pzem) Frequency() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.frequency, nil
}

func (p *pzem) PowerFactor() (float64, error) {
	if err := p.UpdateValues(); err != nil {
		return 0.0, err
	}
	return p.power_factor, nil
}
