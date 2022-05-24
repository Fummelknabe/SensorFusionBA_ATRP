using PyCall

pysockets = pyimport("socket")

export HOST, PORT
const HOST = "192.168.0.103"; #"10.42.0.1"
      PORT = 2222

# Create a python socket
export pysocket
pysocket = pysockets.socket(pysockets.AF_INET, pysockets.SOCK_STREAM)
pysocket.settimeout(20)

export connectToJetson
"""
Connects to the Jetson

# Arguments
- `ip::String`: The IP-Address as string to connect to.
- `port::Integer`: The Port of the software to connect to.

# Returns
- `boolean`: True if connection to a valid controller was succesful.
"""
function connectToJetson(ip::String=HOST, port::Integer=PORT)    
    try
        pysocket.connect((ip, port))
    catch error
        if isa(error, Base.IOError)
            @warn "Connection to Jetson Timed Out!"
        else
            print(error)
        end

        pysocket.close()
        return false
    end

    # Connected to something -----
    data = pysocket.recv(1024)
    println(data)

    pysocket.send(b"atsv_controller")

    return data == "atsv_brain_welcome"
end

export sendAndRecvData
"""
This function sends and receives data over the established tcp connection.

# Arguments 
- `data::String`: The data to send as String.

# Returns
- `String`: The data the sockets receives.
"""
function sendAndRecvData(data::String)
    pysocket.send(codeunits(data))

    return pysocket.recv(1024)
end

function checkConnction(ip::String, port::String)    
    try        
        if connectToJetson(ip, parse(Int64, port))
            return "Connection established!"
        end
           
        return "Connection failed, try again!"
    catch error
        if isa(error, ArgumentError)
            @warn "Port or IP incorrect \n Connection to standard port and IP"
            if connectToJetson()
                return "Connection established!"
            end

            return "Connection failed, try again!"
        end

        return "Connection failed, try again!"
    end
end
