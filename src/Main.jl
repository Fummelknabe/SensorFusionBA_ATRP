include("Client.jl")

"""

This is the starting point of the program.
"""
function main()
    println("The program is started! \nTrying to connect to Jetson:")
    
    if connectToJetson(HOST, PORT)
        println("Connection established!")
    end

    # Send a few commands for testing purposes:
    println(sendAndRecvData("Hallo Jetson!"))
end

main()