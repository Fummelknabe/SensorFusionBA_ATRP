using Sockets

export HOST, PORT
const HOST = "10.42.0.1"; 
      PORT = 2222

export connectToJetson
function connectToJetson(ip::String, port::Integer)
    try
        clientSocket = connect(ip, port)
        print(clientSocket)
    catch error
        if isa(error, Base.IOError)
            @warn "Connection to Jetson Timed Out!"
        end
    end
end

