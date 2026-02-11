#ifndef FEHFILE_H
#define FEHFILE_H

#include <SdFat.h>

/**
 * @brief Access to a file on the SD card
 *
 * An instance of this class is returned by `FEHSD::FOpen`.
 */
class FEHFile
{
public:
    SdFile file_ptr;
    static int prevFileId;
    int fileIdNum;

    FEHFile()
    {
        fileIdNum = ++prevFileId;
    }
};
#endif
