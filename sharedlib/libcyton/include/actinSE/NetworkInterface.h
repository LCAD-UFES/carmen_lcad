#ifndef actinSE_NetworkInterface_H_
#define actinSE_NetworkInterface_H_
//     Copyright (c) 2009-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    NetworkInterface.h
//
// Description: Base level networking capability.  Defines the types of
//              messages that can be sent and received as well as the
//              general methods in use for all protocols.
//
// Contents:    actinSE::NetworkInterface class
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"

namespace actinSE
{

template <typename T> struct NetworkInterfaceImpl;

enum ProtocolType
{
   SkypeProtocol,
   TcpProtocol,
   UdpProtocol
};

static const EcU16 Port = 7654;

template <typename T>
class EC_ACTINSE_DECL NetworkInterface
{
public:
   NetworkInterface
      (
      T& impl,
      const ProtocolType protocol
      );

   ~NetworkInterface
      (
      );

   /// Skype-specific mutator
   void setSkypeData
      (
      const EcString& skypeId,
      const EcString& skypeApp
      );

   /// Tcp/Udp mutator
   void setIPData
      (
      const EcString& address,
      const EcU16 port
      );

   /// Server-side method that will listen for incoming connections.
   /// If a callback handler was registered, then it will spawn
   /// a separate thread to handle the connection.  Otherwise it will
   /// block until an incoming request is made.
   /// @return EcBoolean Success or failure of command
   EcBoolean listen
      (
      );

   /// Client-side connection-oriented method that attempts to connect
   /// to a server using previously-supplied or default values.
   /// If a callback handler was registered, then on
   /// success it will spawn a separate thread to handle return messages.
   /// @return EcBoolean Success or failure of command
   EcBoolean connectToHost
      (
      );

   /// Close down and terminate the existing connection (if any).
   /// Cleans up any memory or resources related to the connection.
   void disconnect
      (
      );

   /// For connection-oriented protocols, checks to make sure that the
   /// connection is still up and valid.  For connection-less protocols,
   /// it should simply return EcTrue.
   /// @return EcBoolean Success or failure of command
   EcBoolean isConnected
      (
      );

   /// Retrieve a handle to compatible implementation.
   /// @return T* Pointer to networked version of implementation class
   T* handle
      (
      );

private:
   NetworkInterfaceImpl<T>* m_pImpl;
};

} // namespace actinSE

#endif // actinSE_NetworkInterface_H_
