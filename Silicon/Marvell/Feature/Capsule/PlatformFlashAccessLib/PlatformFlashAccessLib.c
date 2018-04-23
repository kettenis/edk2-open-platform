/** @file
  Platform flash device access library for Socionext SynQuacer

  Copyright (c) 2016, Linaro, Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PlatformFlashAccessLib.h>
#include <Library/UefiBootServicesTableLib.h>


#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/FileHandleLib.h>
#include <Library/HiiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/ShellCommandLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/Spi.h>
#include <Protocol/SpiFlash.h>

#define CMD_NAME_STRING       L"capsuleupdate"
#define MAIN_HDR_MAGIC        0xB105B002

STATIC MARVELL_SPI_FLASH_PROTOCOL *SpiFlashProtocol;
STATIC MARVELL_SPI_MASTER_PROTOCOL *SpiMasterProtocol;

typedef struct {              // Bytes
  UINT32  Magic;              //  0-3
  UINT32  PrologSize;         //  4-7
  UINT32  PrologChecksum;     //  8-11
  UINT32  BootImageSize;      // 12-15
  UINT32  BootImageChecksum;  // 16-19
  UINT32  Reserved0;          // 20-23
  UINT32  LoadAddr;           // 24-27
  UINT32  ExecAddr;           // 28-31
  UINT8   UartConfig;         //  32
  UINT8   Baudrate;           //  33
  UINT8   ExtCount;           //  34
  UINT8   AuxFlags;           //  35
  UINT32  IoArg0;             // 36-39
  UINT32  IoArg1;             // 40-43
  UINT32  IoArg2;             // 43-47
  UINT32  IoArg3;             // 48-51
  UINT32  Reserved1;          // 52-55
  UINT32  Reserved2;          // 56-59
  UINT32  Reserved3;          // 60-63
} MV_FIRMWARE_IMAGE_HEADER;

STATIC
EFI_STATUS
SpiFlashProbe (
  IN SPI_DEVICE    *Slave
  )
{
  EFI_STATUS       Status;

  // Read SPI flash ID
  Status = SpiFlashProtocol->ReadId (Slave, FALSE);
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  Status = SpiFlashProtocol->Init (SpiFlashProtocol, Slave);
  if (EFI_ERROR(Status)) {
    Print (L"%s: Cannot initialize flash device\n", CMD_NAME_STRING);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
CheckImageHeader (
  IN OUT UINTN *ImageHeader
  )
{
  MV_FIRMWARE_IMAGE_HEADER *Header;
  UINT32 HeaderLength, Checksum, ChecksumBackup;

  Header = (MV_FIRMWARE_IMAGE_HEADER *)ImageHeader;
  HeaderLength = Header->PrologSize;
  ChecksumBackup = Header->PrologChecksum;

  // Compare magic number
  if (Header->Magic != MAIN_HDR_MAGIC) {
    Print (L"%s: Bad Image magic 0x%08x != 0x%08x\n", CMD_NAME_STRING, Header->Magic, MAIN_HDR_MAGIC);
    return EFI_DEVICE_ERROR;
  }

  // The checksum field is discarded from calculation
  Header->PrologChecksum = 0;

  Checksum = CalculateSum32 ((UINT32 *)Header, HeaderLength);
  if (Checksum != ChecksumBackup) {
    Print (L"%s: Bad Image checksum. 0x%x != 0x%x\n", CMD_NAME_STRING, Checksum, ChecksumBackup);
    return EFI_DEVICE_ERROR;
  }

  // Restore checksum backup
  Header->PrologChecksum = ChecksumBackup;

  return 0;
}


/**
  Perform flash write operation.

  @param[in] FirmwareType      The type of firmware.
  @param[in] FlashAddress      The address of flash device to be accessed.
  @param[in] FlashAddressType  The type of flash device address.
  @param[in] Buffer            The pointer to the data buffer.
  @param[in] Length            The length of data buffer in bytes.

  @retval EFI_SUCCESS           The operation returns successfully.
  @retval EFI_WRITE_PROTECTED   The flash device is read only.
  @retval EFI_UNSUPPORTED       The flash device access is unsupported.
  @retval EFI_INVALID_PARAMETER The input parameter is not valid.
**/
EFI_STATUS
EFIAPI
PerformFlashWrite (
  IN PLATFORM_FIRMWARE_TYPE       FirmwareType,
  IN EFI_PHYSICAL_ADDRESS         FlashAddress,
  IN FLASH_ADDRESS_TYPE           FlashAddressType,
  IN VOID                         *Buffer,
  IN UINTN                        Length
  )
{
  SPI_DEVICE              *Slave = NULL;
  EFI_STATUS              Status;

  if (FlashAddressType != FlashAddressTypeAbsoluteAddress) {
    DEBUG ((DEBUG_ERROR, "%a: only FlashAddressTypeAbsoluteAddress supported\n",
      __FUNCTION__));

    return EFI_INVALID_PARAMETER;
  }

  if (FirmwareType != PlatformFirmwareTypeSystemFirmware) {
    DEBUG ((DEBUG_ERROR,
      "%a: only PlatformFirmwareTypeSystemFirmware supported\n",
      __FUNCTION__));

    return EFI_INVALID_PARAMETER;
  }

  // Locate SPI protocols
  Status = gBS->LocateProtocol (
                   &gMarvellSpiFlashProtocolGuid,
                   NULL,
                   (VOID **)&SpiFlashProtocol
                 );

  if (EFI_ERROR(Status)) {
    Print (L"%s: Cannot locate SpiFlash protocol\n", CMD_NAME_STRING);
    return EFI_DEVICE_ERROR;
  }

  Status = gBS->LocateProtocol (
                   &gMarvellSpiMasterProtocolGuid,
                   NULL,
                   (VOID **)&SpiMasterProtocol
                 );

  if (EFI_ERROR(Status)) {
    Print (L"%s: Cannot locate SpiMaster protocol\n", CMD_NAME_STRING);
    return EFI_DEVICE_ERROR;
  }

  // Check image checksum and magic
  Status = CheckImageHeader (Buffer);
  if (EFI_ERROR(Status)) {
    goto HeaderError;
  }
  // Setup and probe SPI flash
  Slave = SpiMasterProtocol->SetupDevice (SpiMasterProtocol, Slave, 0, 0);
  if (Slave == NULL) {
    Print(L"%s: Cannot allocate SPI device!\n", CMD_NAME_STRING);
    goto HeaderError;
  }

  Status = SpiFlashProbe (Slave);
  if (EFI_ERROR(Status)) {
    Print (L"%s: Error while performing SPI flash probe\n", CMD_NAME_STRING);
    goto FlashProbeError;
  }

  // Update firmware image in flash at offset 0x0
  Status = SpiFlashProtocol->Update (Slave, 0, Length, (UINT8 *)Buffer);

  // Release resources
  SpiMasterProtocol->FreeDevice(Slave);

  if (EFI_ERROR(Status)) {
    Print (L"%s: Error while performing flash update\n", CMD_NAME_STRING);
    return EFI_DEVICE_ERROR;
  }

  Print (L"%s: Update %d bytes at offset 0x0 succeeded!\n", CMD_NAME_STRING, Length);

  return EFI_SUCCESS;

FlashProbeError:
  SpiMasterProtocol->FreeDevice(Slave);
HeaderError:

  return EFI_DEVICE_ERROR;
}
