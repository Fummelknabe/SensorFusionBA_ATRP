include("Client.jl")
"""

This is the starting point of the program.
"""
function main()
    println("The program is started! \nTrying to connect to Jetson:")
    connectToJetson(HOST, PORT)
end

main()