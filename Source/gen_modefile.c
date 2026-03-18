/*
 * gen_modefile.c — Host tool to generate the USBAUDIO AudioModes file.
 *
 * Produces a binary IFF-AHIM file (big-endian) for AHI.
 *
 * Required structure (from AHI database.c / AddModeFile):
 *
 *   FORM AHIM {
 *       AUDN { "usbaudio\0" }     ← driver name (PropChunk)
 *       AUDM { TagItem[] ... }    ← mode definition (CollectionChunk)
 *       AUDM { TagItem[] ... }    ← mode definition
 *       AUDM { TagItem[] ... }    ← mode definition
 *   }
 *
 * AHI's AddModeFile flow:
 *   1. Finds AUDN → constructs "DEVS:AHI/<audn>.audio", opens driver with
 *      OpenLibrary to verify it exists; sets AHIDB_Driver = audn.
 *   2. Collects all AUDM chunks (CollectionChunk).
 *   3. For each AUDM: iterates TagItem array via NextTagItem().
 *      For relative tags (AHI_TagBaseR bit set, e.g. AHIDB_Name):
 *          tag->ti_Data += (ULONG) ci->ci_Data
 *      (offset from chunk data start → absolute pointer).
 *   4. Calls AHI_AddAudioMode() with AHIDB_Driver + mode tags.
 *
 * Without AUDN the driver is never opened, modes get AHIDB_Driver=0
 * → crash in strcpy when AHI prefs queries mode info.
 *
 * AudioID scheme: 0x0055XYYY
 *   0x55 = "U" for USB Audio
 *   X/YYY = mode index
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* AHI tag values */
#define AHI_TagBase     0x80000000UL
#define AHI_TagBaseR    (AHI_TagBase | 0x8000UL)

#define AHIDB_AudioID   (AHI_TagBase + 100)
#define AHIDB_Volume    (AHI_TagBase + 103)
#define AHIDB_Panning   (AHI_TagBase + 104)
#define AHIDB_Stereo    (AHI_TagBase + 105)
#define AHIDB_HiFi      (AHI_TagBase + 106)
#define AHIDB_MultTable (AHI_TagBase + 108)
#define AHIDB_Name      (AHI_TagBaseR + 109)
#define AHIDB_MultiChannel (AHI_TagBase + 144)

#define TAG_DONE 0

/* ---- IFF helpers ---- */

/* Write a big-endian 32-bit value */
static void write_be32(FILE *f, uint32_t val)
{
    uint8_t b[4];
    b[0] = (val >> 24) & 0xFF;
    b[1] = (val >> 16) & 0xFF;
    b[2] = (val >>  8) & 0xFF;
    b[3] = (val >>  0) & 0xFF;
    fwrite(b, 1, 4, f);
}

/* Write a TagItem pair [tag:4][data:4] */
static void write_tag(FILE *f, uint32_t tag, uint32_t value)
{
    write_be32(f, tag);
    write_be32(f, value);
}

/*
 * Compute the size of one AUDM chunk's data.
 *
 * AUDM layout:
 *   8 TagItem pairs (7 normal + AHIDB_Name) = 64 bytes
 *   1 TAG_DONE pair                          =  8 bytes
 *   NUL-terminated name string, padded to 2  = variable
 *   Total                                    = 72 + name_padded
 */
static size_t audm_data_size(const char *name)
{
    size_t name_len    = strlen(name) + 1;       /* +1 for NUL */
    size_t name_padded = (name_len + 1) & ~1;    /* IFF: pad to even */
    return 9 * 8 + name_padded;                  /* 9 TagItems + string */
}

/*
 * Write one AUDM chunk (header + data).
 */
static void write_audm(FILE *f,
                        uint32_t audio_id,
                        int volume,
                        int panning,
                        int stereo,
                        int hifi,
                        int multichannel,
                        const char *name)
{
    size_t name_len    = strlen(name) + 1;
    size_t name_padded = (name_len + 1) & ~1;
    uint32_t tags_bytes = 9 * 8;              /* 9 TagItem pairs */
    uint32_t chunk_size = tags_bytes + (uint32_t)name_padded;

    /* AUDM chunk header */
    fwrite("AUDM", 1, 4, f);
    write_be32(f, chunk_size);

    /* TagItem array — all are [tag:4][data:4] = 8 bytes */
    write_tag(f, AHIDB_AudioID,   audio_id);
    write_tag(f, AHIDB_Volume,    volume  ? 1 : 0);
    write_tag(f, AHIDB_Panning,   panning ? 1 : 0);
    write_tag(f, AHIDB_Stereo,    stereo  ? 1 : 0);
    write_tag(f, AHIDB_HiFi,     hifi    ? 1 : 0);
    write_tag(f, AHIDB_MultTable, 0);
    write_tag(f, AHIDB_MultiChannel, multichannel ? 1 : 0);

    /* AHIDB_Name: ti_Data = offset from chunk data start to string */
    write_tag(f, AHIDB_Name,     tags_bytes);  /* = 64 */

    /* TAG_DONE */
    write_tag(f, TAG_DONE,       0);

    /* String data at offset 64 */
    fwrite(name, 1, name_len, f);

    /* Pad to even */
    if (name_padded > name_len)
    {
        uint8_t zero = 0;
        fwrite(&zero, 1, 1, f);
    }
}

/* ---- Mode definitions ---- */

struct mode_def {
    uint32_t id;
    int      volume, panning, stereo, hifi, multichannel;
    const char *name;
};

static const struct mode_def modes[] = {
    { 0x00550001, 1, 1, 1, 1, 0, "USB Audio:HiFi 16 bit stereo++"     },
    { 0x00550002, 1, 1, 1, 1, 1, "USB Audio:HiFi 16 bit 7.1 channel++" },
    { 0x00550003, 1, 1, 1, 0, 0, "USB Audio:16 bit stereo"             },
    { 0x00550005, 1, 0, 0, 0, 0, "USB Audio:16 bit mono"               },
};
#define NUM_MODES (sizeof(modes) / sizeof(modes[0]))

/* ---- main ---- */

int main(void)
{
    FILE *f;
    int i;
    const char *driver_name = "usbaudio";     /* → DEVS:AHI/usbaudio.audio */
    size_t drv_len, drv_padded;
    uint32_t audn_chunk_size;
    uint32_t form_body_size;

    f = fopen("USBAUDIO", "wb");
    if (!f) {
        fprintf(stderr, "Cannot create USBAUDIO\n");
        return 1;
    }

    /*
     * Calculate FORM body size:
     *   "AHIM"                              = 4
     *   AUDN chunk: "AUDN"(4) + size(4) + data(padded) = 8 + drv_padded
     *   For each AUDM: "AUDM"(4) + size(4) + data(padded) = 8 + audm_padded
     */
    drv_len    = strlen(driver_name) + 1;    /* +1 for NUL */
    drv_padded = (drv_len + 1) & ~1;         /* IFF even pad */
    audn_chunk_size = (uint32_t)drv_padded;

    form_body_size = 4;                      /* "AHIM" type ID */
    form_body_size += 4 + 4 + audn_chunk_size;  /* AUDN chunk */

    for (i = 0; i < (int)NUM_MODES; i++)
    {
        size_t ds = audm_data_size(modes[i].name);
        size_t ds_padded = (ds + 1) & ~1;
        form_body_size += 4 + 4 + (uint32_t)ds_padded;  /* AUDM chunk */
    }

    /* FORM header */
    fwrite("FORM", 1, 4, f);
    write_be32(f, form_body_size);
    fwrite("AHIM", 1, 4, f);

    /* AUDN chunk — driver base name (NUL-terminated) */
    fwrite("AUDN", 1, 4, f);
    write_be32(f, audn_chunk_size);
    fwrite(driver_name, 1, drv_len, f);
    if (drv_padded > drv_len)
    {
        uint8_t zero = 0;
        fwrite(&zero, 1, 1, f);
    }

    /* AUDM chunks — one per mode */
    for (i = 0; i < (int)NUM_MODES; i++)
    {
        write_audm(f,
                   modes[i].id,
                   modes[i].volume,
                   modes[i].panning,
                   modes[i].stereo,
                   modes[i].hifi,
                   modes[i].multichannel,
                   modes[i].name);
    }

    fclose(f);
    fprintf(stderr, "USBAUDIO: wrote %d modes, driver=\"%s\"\n",
            (int)NUM_MODES, driver_name);
    return 0;
}
