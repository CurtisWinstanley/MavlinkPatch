import os
import re
import sys
from generator import mavgen_cpp11
from generator import mavgen
from generator import mavparse



def main():
    print("Hello World!")
    xml_list = ["testMavMessage.xml"]
    opts = mavgen.Opts(output="./messages",language="c", wire_protocol=mavparse.PROTOCOL_2_0)
    #mavgen_cpp11.generate(opts, xml_list)
    mavgen.mavgen(opts, xml_list)


if __name__ == "__main__":
    main()