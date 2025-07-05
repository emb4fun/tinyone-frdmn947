# README for TinyONE
TinyONE is the implementation of a project which is based on several building blocks. 
The result is a DHCP and SNTP server which can be used for testing other projects. 
For example, in an enterprise environment it is difficult to quickly change the DHCP 
or SNTP server if these functions are to be tested in your own project. For this reason 
the project TinyONE was created.

As in my other Tiny projects, Tiny does not mean small here, only the functionality 
that was currently needed was implemented. And ONE stands for the first project I have 
created here, which connects all the building blocks together in a meaningful way.

The project is no longer in a Proof of Concept status, but it is not yet fully developed 
also. As already mentioned, this is the first version, and there are still some areas of 
work to be done.

More information are available here: 
https://www.emb4fun.de/projects/tone/index.html

# Some notes about Mbed TLS
Mbed TLS is used in .\source\common\library\mbedtls and was copied from the following project:
https://github.com/ARMmbed/mbedtls

# Notes about NXP Software Components

This project uses software components from the MCUXpresso SDK for the FRDM-MCXN947 board by NXP,
including the `els_pkc` module which provides cryptographic functionality.

These components are subject to the LA_OPT_NXP_Software_License and may only be used in connection
with NXP hardware. Redistribution or modification of these components outside this context is not
permitted.

The original directory structure and license file from the MCUXpresso SDK have been preserved.
For more details, refer to: https://www.nxp.com/docs/en/disclaimer/LA_OPT_NXP_SW.html