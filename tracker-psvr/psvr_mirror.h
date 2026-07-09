/* PSVR side-by-side display mirror.
 * When the PSVR is in VR mode it splits its 1920x1080 panel into two
 * 960x1080 halves (one per eye). This helper opens a borderless window
 * on the PSVR display and duplicates the main desktop into both halves
 * so each eye sees the same image.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void psvr_mirror_start(void);
void psvr_mirror_stop(void);

#ifdef __cplusplus
}
#endif
