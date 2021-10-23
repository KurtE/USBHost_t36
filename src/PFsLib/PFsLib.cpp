#include "PFsLib.h"

//Set to 0 for debug info
#define DBG_Print	0
#if defined(DBG_Print)
#define DBGPrintf Serial.printf
#else
void inline DBGPrintf(...) {};
#endif

//------------------------------------------------------------------------------
#define PRINT_FORMAT_PROGRESS 1
#if !PRINT_FORMAT_PROGRESS
#define writeMsg(str)
#elif defined(__AVR__)
#define writeMsg(str) if (m_pr) m_pr->print(F(str))
#else  // PRINT_FORMAT_PROGRESS
#define writeMsg(str) if (m_pr) m_pr->write((const char*)str)
#endif  // PRINT_FORMAT_PROGRESS

//----------------------------------------------------------------
#define SECTORS_2GB 4194304   // (2^30 * 2) / 512
#define SECTORS_32GB 67108864 // (2^30 * 32) / 512
#define SECTORS_127GB 266338304 // (2^30 * 32) / 512

//uint8_t partVols_drive_index[10];

//=============================================================================
bool PFsLib::deletePartition(BlockDeviceInterface *blockDev, uint8_t part, print_t* pr, Stream &Serialx) 
{
  uint8_t  sectorBuffer[512];
  
  m_pr = pr;

  MbrSector_t* mbr = reinterpret_cast<MbrSector_t*>(sectorBuffer);
  if (!blockDev->readSector(0, sectorBuffer)) {
    writeMsg(F("\nERROR: read MBR failed.\n"));
    return false;
  }

  if ((part < 1) || (part > 4)) {
    m_pr->printf(F("ERROR: Invalid Partition: %u, only 1-4 are valid\n"), part);
    return false;
  }

  writeMsg(F("Warning this will delete the partition are you sure, continue: Y? "));
  int ch;
  
  //..... TODO CIN for READ ......
  while ((ch = Serialx.read()) == -1) ;
  if (ch != 'Y') {
    writeMsg(F("Canceled"));
    return false;
  }
  DBGPrintf(F("MBR Before"));
#if(DBG_Print)
	dump_hexbytes(&mbr->part[0], 4*sizeof(MbrPart_t));
#endif
  // Copy in the higher numer partitions; 
  for (--part; part < 3; part++)  memcpy(&mbr->part[part], &mbr->part[part+1], sizeof(MbrPart_t));
  // clear out the last one
  memset(&mbr->part[part], 0, sizeof(MbrPart_t));

  DBGPrintf(F("MBR After"));
#if(DBG_Print)
  dump_hexbytes(&mbr->part[0], 4*sizeof(MbrPart_t));
#endif
  return blockDev->writeSector(0, sectorBuffer);
}

//===========================================================================
//----------------------------------------------------------------
#define SECTORS_2GB 4194304   // (2^30 * 2) / 512
#define SECTORS_32GB 67108864 // (2^30 * 32) / 512
#define SECTORS_127GB 266338304 // (2^30 * 32) / 512

//uint8_t partVols_drive_index[10];

//----------------------------------------------------------------
// Function to handle one MS Drive...
//msc[drive_index].usbDrive()
void PFsLib::InitializeDrive(BlockDeviceInterface *dev, uint8_t fat_type, print_t* pr)
{
  uint8_t  sectorBuffer[512];

  m_dev = dev;
  m_pr = pr;
  
  //TODO: have to see if this is still valid
  PFsVolume partVol;
/*
  for (int ii = 0; ii < count_partVols; ii++) {
    if (partVols_drive_index[ii] == drive_index) {
      while (Serial.read() != -1) ;
      writeMsg(F("Warning it appears like this drive has valid partitions, continue: Y? "));
      int ch;
      while ((ch = Serial.read()) == -1) ;
      if (ch != 'Y') {
        writeMsg(F("Canceled"));
        return;
      }
      break;
    }
  }

  if (drive_index == LOGICAL_DRIVE_SDIO) {
    dev = sd.card();
  } else if (drive_index == LOGICAL_DRIVE_SDSPI) {
    dev = sdSPI.card();
  } else {
    if (!msDrives[drive_index]) {
      writeMsg(F("Not a valid USB drive"));
      return;
    }
    dev = (USBMSCDevice*)msc[drive_index].usbDrive();
  }
*/
  uint32_t sectorCount = dev->sectorCount();
  
  m_pr->printf(F("sectorCount = %u, FatType: %x\n"), sectorCount, fat_type);
  
  // Serial.printf(F("Blocks: %u Size: %u\n"), msDrives[drive_index].msCapacity.Blocks, msDrives[drive_index].msCapacity.BlockSize);
  if ((fat_type == FAT_TYPE_EXFAT) && (sectorCount < 0X100000 )) fat_type = 0; // hack to handle later
  if ((fat_type == FAT_TYPE_FAT16) && (sectorCount >= SECTORS_2GB )) fat_type = 0; // hack to handle later
  if ((fat_type == FAT_TYPE_FAT32) && (sectorCount >= SECTORS_127GB )) fat_type = 0; // hack to handle later
  if (fat_type == 0)  {
    // assume 512 byte blocks here.. 
    if (sectorCount < SECTORS_2GB) fat_type = FAT_TYPE_FAT16;
    else if (sectorCount < SECTORS_32GB) fat_type = FAT_TYPE_FAT32;
    else fat_type = FAT_TYPE_EXFAT;
  }

  // lets generate a MBR for this type...
  memset(sectorBuffer, 0, 512); // lets clear out the area.
  MbrSector_t* mbr = reinterpret_cast<MbrSector_t*>(sectorBuffer);
  setLe16(mbr->signature, MBR_SIGNATURE);

  // Temporary until exfat is setup...
  if (fat_type == FAT_TYPE_EXFAT) {
    m_pr->println(F("TODO createPartition on ExFat"));
    m_dev->writeSector(0, sectorBuffer);
    createExFatPartition(m_dev, 2048, sectorCount, sectorBuffer, &Serial);
    return;
  } else {
    // Fat16/32
    m_dev->writeSector(0, sectorBuffer);
    createFatPartition(m_dev, fat_type, 2048, sectorCount, sectorBuffer, &Serial);
  }
  
	m_dev->syncDevice();
    writeMsg(F("Format Done\r\n"));
 
}


bool PFsLib::formatter(PFsVolume &partVol, uint8_t fat_type, bool dump_drive, bool g_exfat_dump_changed_sectors, Print &Serialx)
{
  uint8_t  sectorBuffer[512];

  m_pr = &Serialx; // I believe we need this as dump_hexbytes prints to this...

  if (fat_type == 0) fat_type = partVol.fatType();

  if (fat_type != FAT_TYPE_FAT12) {
    // 
    uint8_t buffer[512];
    MbrSector_t *mbr = (MbrSector_t *)buffer;
    if (!partVol.blockDevice()->readSector(0, buffer)) return false;
    MbrPart_t *pt = &mbr->part[partVol.part() - 1];

    uint32_t sector = getLe32(pt->relativeSectors);

    // I am going to read in 24 sectors for EXFat.. 
    uint8_t *bpb_area = (uint8_t*)malloc(512*24); 
    if (!bpb_area) {
      writeMsg(F("Unable to allocate dump memory"));
      return false;
    }
    // Lets just read in the top 24 sectors;
    uint8_t *sector_buffer = bpb_area;
    for (uint32_t i = 0; i < 24; i++) {
      partVol.blockDevice()->readSector(sector+i, sector_buffer);
      sector_buffer += 512;
    }

    if (dump_drive) {
      sector_buffer = bpb_area;
      
      for (uint32_t i = 0; i < 12; i++) {
        DBGPrintf(F("\nSector %u(%u)\n"), i, sector);
        dump_hexbytes(sector_buffer, 512);
        sector++;
        sector_buffer += 512;
      }
      for (uint32_t i = 12; i < 24; i++) {
        DBGPrintf(F("\nSector %u(%u)\n"), i, sector);
        compare_dump_hexbytes(sector_buffer, sector_buffer - (512*12), 512);
        sector++;
        sector_buffer += 512;
      }

    } else {  
      if (fat_type != FAT_TYPE_EXFAT) {
        PFsFatFormatter::format(partVol, fat_type, sectorBuffer, &Serialx);
      } else {
        //DBGPrintf(F("ExFatFormatter - WIP\n"));
        PFsExFatFormatter::format(partVol, sectorBuffer, &Serial);
        if (g_exfat_dump_changed_sectors) {
          // Now lets see what changed
          uint8_t *sector_buffer = bpb_area;
          for (uint32_t i = 0; i < 24; i++) {
            partVol.blockDevice()->readSector(sector, buffer);
            DBGPrintf(F("Sector %u(%u)\n"), i, sector);
            if (memcmp(buffer, sector_buffer, 512)) {
              compare_dump_hexbytes(buffer, sector_buffer, 512);
              DBGPrintf("\n");
            }
            sector++;
            sector_buffer += 512;
          }
        }
      }
    }
    free(bpb_area); 
  }
  else {
    writeMsg(F("Formatting of Fat12 partition not supported"));
    return false;
  }
  return true;
}




//================================================================================================
void PFsLib::print_partion_info(PFsVolume &partVol, Stream &Serialx) 
{
  uint8_t buffer[512];
  MbrSector_t *mbr = (MbrSector_t *)buffer;
  if (!partVol.blockDevice()->readSector(0, buffer)) return;
  MbrPart_t *pt = &mbr->part[partVol.part() - 1];

  uint32_t starting_sector = getLe32(pt->relativeSectors);
  uint32_t sector_count = getLe32(pt->totalSectors);
  Serialx.printf(F("Starting Sector: %u, Sector Count: %u\n"), starting_sector, sector_count);    

  FatPartition *pfp = partVol.getFatVol();
  if (pfp) {
    Serialx.printf(F("fatType:%u\n"), pfp->fatType());
    Serialx.printf(F("bytesPerClusterShift:%u\n"), pfp->bytesPerClusterShift());
    Serialx.printf(F("bytesPerCluster:%u\n"), pfp->bytesPerCluster());
    Serialx.printf(F("bytesPerSector:%u\n"), pfp->bytesPerSector());
    Serialx.printf(F("bytesPerSectorShift:%u\n"), pfp->bytesPerSectorShift());
    Serialx.printf(F("sectorMask:%u\n"), pfp->sectorMask());
    Serialx.printf(F("sectorsPerCluster:%u\n"), pfp->sectorsPerCluster());
    Serialx.printf(F("sectorsPerFat:%u\n"), pfp->sectorsPerFat());
    Serialx.printf(F("clusterCount:%u\n"), pfp->clusterCount());
    Serialx.printf(F("dataStartSector:%u\n"), pfp->dataStartSector());
    Serialx.printf(F("fatStartSector:%u\n"), pfp->fatStartSector());
    Serialx.printf(F("rootDirEntryCount:%u\n"), pfp->rootDirEntryCount());
    Serialx.printf(F("rootDirStart:%u\n"), pfp->rootDirStart());
  }
} 


uint32_t PFsLib::mbrDmp(BlockDeviceInterface *blockDev, uint32_t device_sector_count, Stream &Serialx) {
  MbrSector_t mbr;
  m_pr = &Serialx;
  bool gpt_disk = false;
  uint32_t next_free_sector = 8192;  // Some inital value this is default for Win32 on SD...
  // bool valid = true;
  if (!blockDev->readSector(0, (uint8_t*)&mbr)) {
    Serialx.print(F("\nread MBR failed.\n"));
    //errorPrint();
    return (uint32_t)-1;
  }
  Serialx.print(F("\nmsc # Partition Table\n"));
  Serialx.print(F("\tpart,boot,bgnCHS[3],type,endCHS[3],start,length\n"));
  for (uint8_t ip = 1; ip < 5; ip++) {
    MbrPart_t *pt = &mbr.part[ip - 1];
    uint32_t starting_sector = getLe32(pt->relativeSectors);
    uint32_t total_sector = getLe32(pt->totalSectors);
    if (starting_sector > next_free_sector) {
      Serialx.printf(F("\t < unused area starting at: %u length %u >\n"), next_free_sector, starting_sector-next_free_sector);
    }
    switch (pt->type) {
    case 4:
    case 6:
    case 0xe:
      Serialx.print(F("FAT16:\t"));
      break;
    case 11:
    case 12:
      Serialx.print(F("FAT32:\t"));
      break;
    case 7:
      Serialx.print(F("exFAT:\t"));
      break;
    case 0xf:
      Serial.print(F("Extend:\t"));
      break;
    case 0x83: Serialx.print(F("ext2/3/4:\t")); break; 
    case 0xee: 
      Serialx.print(F("*** GPT Disk not supported ***\nGPT guard:\t")); 
      gpt_disk = true;
      break;
    default:
      Serialx.print(F("pt_#"));
      Serialx.print(pt->type);
      Serialx.print(":\t");
      break;
    }
    Serialx.print( int(ip)); Serial.print( ',');
    Serialx.print(int(pt->boot), HEX); Serial.print( ',');
    for (int i = 0; i < 3; i++ ) {
      Serialx.print("0x"); Serial.print(int(pt->beginCHS[i]), HEX); Serial.print( ',');
    }
    Serialx.print("0x"); Serial.print(int(pt->type), HEX); Serial.print( ',');
    for (int i = 0; i < 3; i++ ) {
      Serialx.print("0x"); Serial.print(int(pt->endCHS[i]), HEX); Serial.print( ',');
    }
    Serialx.print(starting_sector, DEC); Serial.print(',');
    Serialx.println(total_sector);

    // Lets get the max of start+total
    if (starting_sector && total_sector)  next_free_sector = starting_sector + total_sector;
  }
  if ((device_sector_count != (uint32_t)-1) && (next_free_sector < device_sector_count)) {
    Serialx.printf(F("\t < unused area starting at: %u length %u >\n"), next_free_sector, device_sector_count-next_free_sector);
  } 
  if (gpt_disk) gptDmp(blockDev, Serialx);
  return next_free_sector;
}

typedef struct {
  uint32_t  q1;
  uint16_t  w2; 
  uint16_t  w3;
  uint8_t   b[8];
} guid_t;

typedef struct {
  uint8_t   signature[8];
  uint32_t  revision;
  uint32_t  crc32;
  uint32_t  reserved;
  uint64_t  currentLBA;
  uint64_t  backupLBA;
  uint64_t  firstLBA;
  uint64_t  lastLBA;
  uint8_t   diskGUID[16];
  uint64_t  startLBAArray;
  uint32_t  numberPartitions;
  uint32_t  sizePartitionEntry;
  uint32_t  crc32PartitionEntries;
  uint8_t   unused[420]; // should be 0;
} gptPartitionHeader_t;

typedef struct {
  uint8_t   partitionTypeGUID[16];
  uint8_t   uniqueGUID[16];
  uint64_t  firstLBA;
  uint64_t  lastLBA;
  uint64_t  attributeFlags;
  uint16_t  name[36];
} gptPartitionEntryItem_t;

typedef struct {
  gptPartitionEntryItem_t items[4];
} gptPartitionEntrySector_t;

void printGUID(uint8_t* pbguid, Stream &Serialx) {
  // Windows basic partion guid is: EBD0A0A2-B9E5-4433-87C0-68B6B72699C7
  // raw dump of it: A2 A0 D0 EB E5 B9 33 44 87 C0 68 B6 B7 26 99 C7
  guid_t *pg = (guid_t*)pbguid;
  Serialx.printf("%08X-%04X-%04X-%02X%-2X-", pg->q1, pg->w2, pg->w3, pg->b[0], pg->b[1]);
  for (uint8_t i=2;i<8; i++) Serialx.printf("%02X", pg->b[i]);
}

static const uint8_t mbdpGuid[16] PROGMEM = {0xA2, 0xA0, 0xD0, 0xEB, 0xE5, 0xB9, 0x33, 0x44, 0x87, 0xC0, 0x68, 0xB6, 0xB7, 0x26, 0x99, 0xC7};

//----------------------------------------------------------------
uint32_t PFsLib::gptDmp(BlockDeviceInterface *blockDev, Stream &Serialx) {
  union {
    MbrSector_t mbr;
    partitionBootSector pbs;
    gptPartitionHeader_t gpthdr;
    gptPartitionEntrySector_t gptes;
    uint8_t buffer[512];
  } sector; 
  m_pr = &Serialx;

  // Lets verify that we are an GPT...
  if (!blockDev->readSector(0, (uint8_t*)&sector.mbr)) {
    Serialx.print(F("\nread MBR failed.\n"));
    //errorPrint();
    return (uint32_t)-1;
  }
  // verify that the first partition is the guard...
  MbrPart_t *pt = &sector.mbr.part[0];
  if (pt->type != 0xee) {
    Serialx.print(F("\nMBR is not an gpt guard\n"));
    return (uint32_t)-1;
  }

  if (!blockDev->readSector(1, (uint8_t*)&sector.buffer)) {
    Serialx.print(F("\nread Partition Table Header failed.\n"));
    return (uint32_t)-1;
  }
  // Do quick test for signature:
  if (memcmp(sector.gpthdr.signature, "EFI PART", 8)!= 0) { 
    Serialx.println("GPT partition header signature did not match");
    dump_hexbytes(&sector.buffer, 512);
  }
  Serialx.printf("\nGPT partition header revision: %x\n", sector.gpthdr.revision);
  Serialx.printf("LBAs current:%llu backup:%llu first:%llu last:%llu\nDisk GUID:", 
    sector.gpthdr.currentLBA, sector.gpthdr.backupLBA, sector.gpthdr.firstLBA, sector.gpthdr.lastLBA);
  printGUID(sector.gpthdr.diskGUID, Serialx);
  //dump_hexbytes(&sector.gpthdr.diskGUID, 16);
  uint32_t cParts = sector.gpthdr.numberPartitions;
  Serialx.printf("Start LBA Array: %llu Count: %u size:%u\n", 
      sector.gpthdr.startLBAArray, cParts, sector.gpthdr.sizePartitionEntry);
  uint32_t sector_number = 2;
  Serialx.println("Part\t Type Guid, Unique Guid, First, last, attr, name");
  for (uint8_t part = 0; part < cParts ; part +=4) {
    if (blockDev->readSector(sector_number, (uint8_t*)&sector.buffer)) {
      //dump_hexbytes(&sector.buffer, 512);
      for (uint8_t ipei = 0; ipei < 4; ipei++) {
        gptPartitionEntryItem_t *pei = &sector.gptes.items[ipei];
        // see if the entry has any data in it...
        uint32_t end_addr = (uint32_t)pei + sizeof(gptPartitionEntryItem_t);
        uint32_t *p = (uint32_t*)pei;
        for (; (uint32_t)p < end_addr; p++) {
          if (*p) break; // found none-zero. 
        }
        if ((uint32_t)p < end_addr) {
          // So entry has data:
          Serialx.printf("%u\t", part + ipei);
          printGUID(pei->partitionTypeGUID, Serialx);
          Serialx.print(", ");
          printGUID(pei->uniqueGUID, Serialx);
          Serialx.printf(", %llu, %llu, %llX, ", pei->firstLBA, pei->lastLBA, pei->attributeFlags);
          for (uint8_t i = 0; i < 36; i++) {
            if ((pei->name[i]) == 0) break;
            Serialx.write((uint8_t)pei->name[i]);
          }
          Serialx.println();
          if (memcmp((uint8_t *)pei->partitionTypeGUID, mbdpGuid, 16) == 0) {
            Serialx.print(">>> Microsoft Basic Data Partition\n");
            // See if we can read in the first sector
            if (blockDev->readSector(pei->firstLBA, (uint8_t*)&sector.buffer)) {
              dump_hexbytes(sector.buffer, 512);

              // First see if this is exFat... 
              // which starts with: 
              static const uint8_t exfatPBS[] PROGMEM = {0xEB, 0x76, 0x90, //Jmp instruction
                   'E', 'X', 'F', 'A', 'T', ' ', ' ', ' '};
              if (memcmp(sector.buffer, exfatPBS, 11) == 0) {
                Serial.println("    EXFAT:");
              }

            }
            // Bugbug reread that sector...
            blockDev->readSector(sector_number, (uint8_t*)&sector.buffer);
          }
        }
      }
    }
    sector_number++;
  }
  return 0;
}
//----------------------------------------------------------------

void PFsLib::dump_hexbytes(const void *ptr, int len)
{
  if (ptr == NULL || len <= 0) return;
  if (m_pr == nullptr) return;
  const uint8_t *p = (const uint8_t *)ptr;
  while (len > 0) {
    for (uint8_t i = 0; i < 32; i++) {
      if (i > len) break;
      m_pr->printf("%02X ", p[i]);
    }
    m_pr->print(":");
    for (uint8_t i = 0; i < 32; i++) {
      if (i > len) break;
      m_pr->printf("%c", ((p[i] >= ' ') && (p[i] <= '~')) ? p[i] : '.');
    }
    m_pr->println();
    p += 32;
    len -= 32;
  }
}

void PFsLib::compare_dump_hexbytes(const void *ptr, const uint8_t *compare_buf, int len)
{
  if (ptr == NULL || len <= 0) return;
  const uint8_t *p = (const uint8_t *)ptr;
  while (len) {
    for (uint8_t i = 0; i < 32; i++) {
      if (i > len) break;
      m_pr->printf("%c%02X", (p[i]==compare_buf[i])? ' ' : '*',p[i]);
    }
    m_pr->print(":");
    for (uint8_t i = 0; i < 32; i++) {
      if (i > len) break;
      m_pr->printf("%c", ((p[i] >= ' ') && (p[i] <= '~')) ? p[i] : '.');
    }
    m_pr->println();
    p += 32;
    compare_buf += 32;
    len -= 32;
  }
}
