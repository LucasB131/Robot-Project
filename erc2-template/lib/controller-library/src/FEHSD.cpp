#include "FEH.h"

#include "../private_include/FEHInternal.h"

SdFat FAT;

FEHSD SD;

uint8_t f_res;

bool inAppendMode = false;

int FEHFile::prevFileId = 0;

FEHSD::FEHSD()
{
    numberOfFiles = 0;
}

FEHFile *FEHSD::FOpen(const char *str, const char *mode)
{
    oflag_t oflag;
    FEHFile *File = new FEHFile();

    // Choosing the appropriate access mode
    if (strcmp(mode, "r") == 0)
    {
        oflag = O_READ;
    }
    else if (strcmp(mode, "r+") == 0)
    {
        oflag = O_RDWR;
    }
    else if (strcmp(mode, "w") == 0)
    {
        oflag = O_CREAT | O_TRUNC | O_WRITE;
    }
    else if (strcmp(mode, "w+") == 0)
    {
        oflag = O_CREAT | O_TRUNC | O_RDWR;
    }
    else if (strcmp(mode, "a") == 0)
    {
        oflag = O_CREAT | O_APPEND | O_WRITE;
        inAppendMode = true;
    }
    else if (strcmp(mode, "a+") == 0)
    {
        oflag = O_CREAT | O_APPEND | O_WRITE | O_READ;
        inAppendMode = true;
    }
    else if (strcmp(mode, "wx") == 0)
    {
        oflag = O_CREAT | O_EXCL | O_WRITE;
    }
    else if (strcmp(mode, "w+x") == 0)
    {
        oflag = O_CREAT | O_EXCL | O_RDWR;
    }
    else
    {
        oflag = O_CREAT | O_TRUNC | O_WRITE;
    }

    f_res = (File->file_ptr).open(str, oflag);

    if (f_res == 0)
    {
        LCD.WriteLine("File failed to open");
        return NULL;
    }
    else if (inAppendMode)
    {
        (File->file_ptr).seekSet((File->file_ptr).fileSize()); // go to the end of the file to append
    }

    files[numberOfFiles++] = File;

    return File;
}

int FEHSD::FClose(FEHFile *fptr)
{
    int i, j;
    if (fptr != NULL)
    {
        for (i = 0; i < numberOfFiles; i++)
        {
            if (fptr->fileIdNum == (files[i])->fileIdNum)
            {
                if (files[i]->file_ptr.isOpen())
                {
                    (files[i]->file_ptr).close(); // I banish thy memoryleaks
                }
                // Shift all elements in array one over to the left
                for (j = i; j < numberOfFiles - 1; j++)
                {
                    files[j] = files[j + 1];
                }
                files[numberOfFiles - 1] = NULL;

                numberOfFiles--;
                break;
            }
        }
    }
    return 0;
}

int FEHSD::FCloseAll()
{
    int i;
    for (i = 0; i < numberOfFiles; i++)
    {
        if (files[i] != NULL)
        {
            if (files[i]->file_ptr.isOpen())
            {
                (files[i]->file_ptr).close(); // I banish thy memoryleaks
            }
            files[i] = NULL;
        }
    }
    numberOfFiles = 0;
    return 0;
}

int FEHSD::FEof(FEHFile *fptr)
{
    return (fptr->file_ptr).available64() > 0;
}

int FEHSD::FPrintf(FEHFile *fptr,
                   const char *str, /* Pointer to the format string */
                   ...)
{
    va_list args;
    va_start(args, str);
    char buffer[BUFFER_SIZE];

    vsnprintf(buffer, BUFFER_SIZE, str, args);

    va_end(args);

    int numChars = (fptr->file_ptr).write(buffer);

    if (numChars <= 0)
    {
        LCD.WriteLine("Error printing to file");
        return -1;
    }
    // Return number of characters printed
    return numChars;
}

int FEHSD::FScanf(FEHFile *fptr, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    // Check for end of file, return -1 if eof
    if (FEof(fptr) == 0)
    {
        LCD.WriteLine("Reached end of file");
        return -1;
    }

    char buffer[BUFFER_SIZE];

    int i = 0;
    while ((fptr->file_ptr).isOpen() && i < BUFFER_SIZE)
    {
        char c = (fptr->file_ptr).read();
        if (c == EOF || c == '\n' || c == '\r')
        {
            buffer[i] = '\0';
            break;
        }
        buffer[i] = c;
        i++;
    }

    //  Scan line and store in args; also get number of args read
    int numRead = my_vsscanf(buffer, format, args);

    va_end(args);

    if (numRead == -1)
    {
        LCD.WriteLine("Error reading from file");
        return -1;
    }

    // Return number of successful reads
    return numRead;
}

int FEHSD::FSeek(FEHFile *fptr, long int offset, int position)
{
    if (position == SEEK_CUR)
    {
        return (fptr->file_ptr).seekSet((fptr->file_ptr).curPosition() + offset);
    }
    else if (position == SEEK_END)
    {
        return (fptr->file_ptr).seekSet((fptr->file_ptr).fileSize() - offset);
    }
    else
    {
        return (fptr->file_ptr).seekSet(offset);
    }
}

void FEHSD::FlushToConsole(const char *str)
{
    SdFile file;
    if (!FAT.exists(str))
    {
        Serial.print(str);
        Serial.print(" does not exist on SD card.");
    }
    else if (file.open(str, O_READ))
    {
        char message[100];
        strcpy(message, "Printing contents of ");
        strcat(message, str);
        Serial.println(message);

        while (file.available64())
        {
            char data = file.read();
            Serial.print(data);
        }
        file.close();
    }
    else
    {
        char message[50];
        strcpy(message, "Failed to open ");
        strcat(message, str);
        Serial.println(message);
    }
}

void FEHSD::FlushToConsole(FEHFile *fptr)
{
    if ((fptr->file_ptr).isReadable())
    {
        while ((fptr->file_ptr).available64())
        {
            char data = (fptr->file_ptr).read();
            Serial.print(data);
        }
    }
    else
    {
        Serial.println("FEHFile given is not open for reading");
    }
}

// HELPER FUNCTIONS

int FEHSD::my_vsscanf(const char *str, const char *format, va_list args)
{
    int count = 0;
    const char *s = str;

    int formatLevel = 0;

    while (strlen(format) > 0 && strlen(s) > 0)
    {

        if (isspace(*format))
        {
            format++;
            continue;
        }

        if (*format == '%' || formatLevel)
        {
            format++; // Move past the '%'
            switch (*format)
            {
            case 'd':
            case 'i':
            {
                // strtol assumes first character is a digit so skip string until that point
                while ((strlen(s) > 0 && !isdigit(s[0])) || (strlen(s) > 1 && s[0] == '-' && !isdigit(s[1])))
                {
                    s++;
                }
                if (strlen(s) > 0)
                {
                    // Integer conversion
                    void *i;
                    if (formatLevel == 0)
                    {
                        i = va_arg(args, int *);
                        *(int *)i = strtol(s, (char **)&s, 10);
                    }
                    else
                    {
                        i = va_arg(args, long *);
                        *(long *)i = strtol(s, (char **)&s, 10);
                    }
                    count++;
                }
                formatLevel = 0;
                break;
            }
            case 'u':
                // strtol assumes first character is a digit so skip string until that point
                while ((strlen(s) > 0 && !isdigit(s[0])))
                {
                    s++;
                }
                if (strlen(s) > 0)
                {
                    // Integer conversion
                    void *i;
                    if (formatLevel == 0)
                    {
                        i = va_arg(args, unsigned int *);
                        *(unsigned int *)i = strtol(s, (char **)&s, 10);
                    }
                    else
                    {
                        i = va_arg(args, unsigned long *);
                        *(unsigned long *)i = strtol(s, (char **)&s, 10);
                    }
                    count++;
                }
                formatLevel = 0;
                break;
            case 'l':
                format++;
                formatLevel++;
                break;
            case 'f':
            {
                // strtod assumes first character is a digit so skip string until that point
                while (
                    (strlen(s) > 0 && !isdigit(s[0])) ||
                    (strlen(s) > 1 && (s[0] == '-' || s[0] == '.') && !isdigit(s[1])) ||
                    (strlen(s) > 2 && s[0] == '-' && s[1] == '.' && !isdigit(s[2])))
                {
                    s++;
                }
                if (strlen(s) > 0)
                {
                    void *f;

                    if (formatLevel == 0)
                    {
                        f = va_arg(args, float *);
                        *(float *)f = strtod(s, (char **)&s);
                    }
                    else
                    {
                        f = va_arg(args, double *);
                        *(double *)f = strtod(s, (char **)&s);
                    }
                    count++;
                }
                formatLevel = 0;
                break;
            }
            case 's':
            {
                char *strArg = va_arg(args, char *);
                while (strlen(s) > 0 && !isspace(*s))
                {
                    *strArg++ = *s++;
                }
                *strArg = '\0';
                count++;
                formatLevel = 0;
                break;
            }
            case 'c':
            {
                char *strArg = va_arg(args, char *);
                if (strlen(s) > 0)
                {
                    *strArg++ = *s++;
                }
                count++;
                formatLevel = 0;
                break;
            }
            default:
                formatLevel = 0;
                LCD.WriteLine("***Unknown format string found.");
                // Handle unknown format
                break;
            }
        }
        else
        {
            while (isspace(*s))
            {
                s++;
            }

            // Skip over non-format characters in the format string
            if (*s == *format)
            {
                s++;
            }
            format++;
        }
    }
    return count;
}
