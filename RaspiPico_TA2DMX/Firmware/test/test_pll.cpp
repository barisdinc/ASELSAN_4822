// PLL kelime hesabi dogrulamasi (host derlemesi: g++ test/test_pll.cpp)
// pllWriteWords() icindeki iki satirin birebir kopyasi.
// Amac: R=1024 (12.8MHz/12.5kHz) ile N/A'nin TAMSAT float formuluyle
// tum bantta ayni sonucu verdigini ve A<80 / N<1024 sinirlarini kanitlamak.
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int N_of(unsigned long f) { return (int)(f / 1000); }
static int A_of(unsigned long f) { return (int)(((f * 2UL) / 25UL) - 80UL * (f / 1000)); }

// TAMSAT (Arduino Nano) referansi: float bolme
static int N_ref(unsigned long f) { return (int)(f / 12.5 / 80); }
static int A_ref(unsigned long f) { return (int)(f / 12.5) - 80 * (int)(f / 12.5 / 80); }

int main(void) {
    int fail = 0, checked = 0;

    // 1) VHF: 134.000 - 174.000 MHz, 25 kHz izgara, RX (LO=+45MHz) ve TX
    for (unsigned long f = 134000; f <= 174000; f += 25) {
        for (int tx = 0; tx < 2; tx++) {
            unsigned long vco = tx ? f : f + 45000UL;
            int N = N_of(vco), A = A_of(vco);
            checked++;
            if (N != N_ref(vco) || A != A_ref(vco)) {
                printf("FAIL math  %lu tx=%d N=%d/%d A=%d/%d\n", f, tx, N, N_ref(vco), A, A_ref(vco));
                fail++;
            }
            if (A < 0 || A >= 80) {   // prescaler siniri
                printf("FAIL A out of range %lu -> A=%d\n", f, A);
                fail++;
            }
            if (N < 0 || N > 1023) {  // 10 bit N
                printf("FAIL N out of range %lu -> N=%d\n", f, N);
                fail++;
            }
            // VCO'ya geri donus: (N*80+A)*12.5 == vco
            double back = (double)(N * 80 + A) * 12.5;
            if (fabs(back - (double)vco) > 0.001) {
                printf("FAIL roundtrip %lu tx=%d -> %.3f\n", f, tx, back);
                fail++;
            }
        }
    }

    // 2) UHF: 430.000 - 470.000 MHz, 25 kHz, RX (LO=-45MHz) ve TX
    for (unsigned long f = 430000; f <= 470000; f += 25) {
        for (int tx = 0; tx < 2; tx++) {
            unsigned long vco = tx ? f : f - 45000UL;
            int N = N_of(vco), A = A_of(vco);
            checked++;
            if (N != N_ref(vco) || A != A_ref(vco)) {
                printf("FAIL UHF math %lu tx=%d\n", f, tx);
                fail++;
            }
            if (A < 0 || A >= 80 || N > 1023) {
                printf("FAIL UHF range %lu N=%d A=%d\n", f, N, A);
                fail++;
            }
        }
    }

    // 3) Bildirilen ariza frekanslari: beklenen N/A degerleri
    struct { unsigned long f; int txN, txA; } cases[] = {
        {145550, 145, 44}, {145575, 145, 46}, {145600, 145, 48},
        {145625, 145, 50}, {145650, 145, 52},
    };
    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        int N = N_of(cases[i].f), A = A_of(cases[i].f);
        checked++;
        if (N != cases[i].txN || A != cases[i].txA) {
            printf("FAIL case %lu -> N=%d A=%d beklenen %d/%d\n",
                   cases[i].f, N, A, cases[i].txN, cases[i].txA);
            fail++;
        }
    }

    printf("%d kontrol, %d hata\n", checked, fail);
    return fail ? EXIT_FAILURE : EXIT_SUCCESS;
}
