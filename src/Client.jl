using Sockets

export HOST, PORT
const HOST = ip"192.168.0.103"; #"10.42.0.1"
      PORT = 2222


export connectToJetsoncl
function connectToJetson(ip::IPv4, port::Integer)
    try
        clientSocket = connect(ip, port)

        @async while !eof(clientSocket)
            readline(clientSocket, keep=true)
            println(clientSocket, "Test Message!")
            println(stdout, "Are we here3?")
        end
    catch error
        if isa(error, Base.IOError)
            @warn "Connection to Jetson Timed Out!"
        else
            print(error)
        end
    end
end

