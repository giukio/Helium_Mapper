// ttn application function to decode uplink data.
// Decode decodes an array of bytes into an object.
//  - port contains the LoRaWAN fPort number
//  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
// The function must return an object, e.g. {"temperature": 22.5}
function Decoder(bytes, port) {
    var decoded = {};
    var hexString=bin2HexStr(bytes);
    return rakSensorDataDecode(hexString);
  }
  
  // convert array of bytes to hex string.
  // e.g: 026700C80371FFCBFE9803A10802019201881A6F893106F7FEBC00002EEA01900301910E7401890275
  function bin2HexStr(bytesArr) {
    var str = "";
    for(var i=0; i<bytesArr.length; i++) {
      var tmp = (bytesArr[i] & 0xff).toString(16);
      if(tmp.length == 1) {
        tmp = "0" + tmp;
      }
      str += tmp;
    }
    return str;
  }
  
  // convert string to short integer
  function parseShort(str, base) {
    var n = parseInt(str, base);
    return (n << 16) >> 16;
  }
  
  // convert string to triple bytes integer
  function parseTriple(str, base) {
    var n = parseInt(str, base);
    return (n << 8) >> 8;
  }
  
  // convert string to Quadruple bytes integer
  function parseQuadruple(str, base) {
    var n = parseInt(str, base);
    return (n << 32) >> 32;
  }
  
  // decode Hex sensor string data to object
  function rakSensorDataDecode(hexStr) {
    var str = hexStr;
    var myObj = {};
  
    while (str.length > 4) {
      var flag = parseInt(str.substring(0, 4), 16);
      switch (flag) {
        case 0x0768:// Humidity
          myObj.humidity = parseFloat(((parseShort(str.substring(4, 6), 16) * 0.01 / 2) * 100).toFixed(1));// + "%RH";//unit:%RH
          str = str.substring(6);
          break;
        case 0x0673:// Atmospheric pressure
          myObj.barometer = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2));// + "hPa";//unit:hPa
          str = str.substring(8);
          break;
        case 0x0267:// Temperature
          myObj.temperature = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2));// + "°C";//unit: °C
          str = str.substring(8);
          break;
  //      case 0x0188:// GPS
  //        var latitude = parseFloat((parseTriple(str.substring(4, 10), 16) * 0.0001).toFixed(4));
  //        var longitude = parseFloat((parseTriple(str.substring(10, 16), 16) * 0.0001).toFixed(4));
  //        myObj.altitude = parseFloat((parseTriple(str.substring(16, 22), 16) * 0.01).toFixed(1)); // Meter
  //        myObj.location = "(" + latitude + "," + longitude + ")";
  //        str = str.substring(22);
  //        break;
        case 0x0188:// GPS
           var latitude = parseFloat((parseQuadruple(str.substring(4, 12), 16) * 0.0000001).toFixed(7));//unit:°
           var longitude = parseFloat((parseQuadruple(str.substring(12, 20), 16) * 0.0000001).toFixed(7));//unit:°
           myObj.altitude = parseFloat((parseQuadruple(str.substring(20, 28), 16) * 0.01).toFixed(1));//unit:m
           myObj.location = "(" + latitude + "," + longitude + ")";
           str = str.substring(28);
           break;
        case 0x0189:// speed kmh
          myObj.speedkph = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// unit:Km/h
          str = str.substring(8);
          break;
        case 0x0190:// satellites
          myObj.sats = parseFloat((parseShort(str.substring(4, 6), 16)).toFixed(0));
          str = str.substring(6);
          break;
        case 0x0191:// hdop
          myObj.hdop = parseFloat((parseShort(str.substring(4, 8), 16) * 0.001).toFixed(3));
          str = str.substring(8);
          break;
        case 0x0371:// Triaxial acceleration
          myObj.acceleration_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.001).toFixed(3));
          myObj.acceleration_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.001).toFixed(3));
          myObj.acceleration_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.001).toFixed(3)); //g
          str = str.substring(16);
          break;
        case 0x0402:// air resistance
          myObj.gasResistance = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "KΩ";//unit:KΩ
          str = str.substring(8);
          break;
        case 0x0802:// Battery Voltage
          myObj.battery = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "V";//unit:V
          str = str.substring(8);
          break;
        case 0x0586:// gyroscope
          myObj.gyroscope_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "°/s";//unit:°/s
          myObj.gyroscope_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.01).toFixed(2));// + "°/s";//unit:°/s
          myObj.gyroscope_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.01).toFixed(2));// + "°/s";//unit:°/s
          str = str.substring(16);
          break;
        case 0x0902:// magnetometer x
          myObj.magnetometer_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "μT";//unit:μT
          str = str.substring(8);
          break;
        case 0x0a02:// magnetometer y
          myObj.magnetometer_y = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "μT";//unit:μT
          str = str.substring(8);
          break;
        case 0x0b02:// magnetometer z
          myObj.magnetometer_z = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2));// + "μT";//unit:μT
          str = str.substring(8);
          break;
        default:
          str = str.substring(7);
          break;
      }
    }
  
    var decoded = myObj;
    
      if (normalizedPayload.gateways[0].rssi) {
          decoded.LORA_RSSI = normalizedPayload.gateways[0].rssi;
      }
      
      if (normalizedPayload.gateways[0].snr) {
          decoded.LORA_SNR = normalizedPayload.gateways[0].snr;
      }
      
      if (normalizedPayload.data_rate) {
          decoded.LORA_DATARATE = normalizedPayload.data_rate;
      }   
      
      // Array where we store the fields that are being sent to Datacake
      var datacakeFields = []
      
      // take each field from decodedElsysFields and convert them to Datacake format
      for (var key in decoded) {
          if (decoded.hasOwnProperty(key)) {           
              datacakeFields.push({field: key.toUpperCase(), value: decoded[key]})
          }
      }      
      
      // forward data to Datacake
      return datacakeFields;    
  }