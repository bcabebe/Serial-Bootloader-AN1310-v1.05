#ifdef __12LF1840
     #include p12LF1840.inc

     #define DEVICEID                .222
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

; PIC12F609 does not support self-programming.
; PIC12F615 does not support self-programming.
#ifdef __12F617
     #include p12F617.inc

     #define DEVICEID                .155
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x100
#endif

#ifdef __12F629
     #include p12F629.inc

     #define DEVICEID                .124
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xE0
#endif

#ifdef __12F635
     #include p12F635.inc

     #define DEVICEID                .125
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __12F675
     #include p12F675.inc

     #define DEVICEID                .126
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xE0
#endif

#ifdef __12F683
     #include p12F683.inc

     #define DEVICEID                .35
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x100
#endif

#ifdef __12F752
     #include p12F752.inc

     #define DEVICEID                .168
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __12F1822
     #include p12F1822.inc

     #define DEVICEID                .312
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __12F1840
     #include p12F1840.inc

     #define DEVICEID                .220
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

; PIC12HV609 does not support self-programming.
; PIC12HV615 does not support self-programming.
#ifdef __12HV752
     #include p12HV752.inc

     #define DEVICEID                .169
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __12LF1822
     #include p12LF1822.inc

     #define DEVICEID                .320
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1947
     #include p16LF1947.inc

     #define DEVICEID                .301
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F72
     #include p16F72.inc

     #define DEVICEID                .5
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F73
     #include p16F73.inc

     #define DEVICEID                .48
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F74
     #include p16F74.inc

     #define DEVICEID                .49
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F76
     #include p16F76.inc

     #define DEVICEID                .50
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F77
     #include p16F77.inc

     #define DEVICEID                .51
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F83
     #include p16F83.inc

     #define DEVICEID                .51
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x200
     #define END_GPR                 0xB0
#endif

#ifdef __16F84
     #include p16F84.inc

     #define DEVICEID                .51
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xD0
#endif

#ifdef __16F84A
     #include p16F84A.inc

     #define DEVICEID                .43
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xD0
#endif

#ifdef __16F87
     #include p16F87.inc

     #define DEVICEID                .57
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F88
     #include p16F88.inc

     #define DEVICEID                .59
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

; PIC16F610 does not support self-programming.
; PIC16F616 does not support self-programming.
#ifdef __16F627
     #include p16F627.inc

     #define DEVICEID                .61
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __16F627A
     #include p16F627A.inc

     #define DEVICEID                .130
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __16F628
     #include p16F628.inc

     #define DEVICEID                .62
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F628A
     #include p16F628A.inc

     #define DEVICEID                .131
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F630
     #include p16F630.inc

     #define DEVICEID                .134
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xE0
#endif

#ifdef __16F631
     #include p16F631.inc

     #define DEVICEID                .161
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __16F636
     #include p16F636.inc

     #define DEVICEID                .133
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F639
     #include p16F639.inc

     #define DEVICEID                .133
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F648A
     #include p16F648A.inc

     #define DEVICEID                .136
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F676
     #include p16F676.inc

     #define DEVICEID                .135
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x400
     #define END_GPR                 0xE0
#endif

#ifdef __16F677
     #include p16F677.inc

     #define DEVICEID                .162
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F684
     #include p16F684.inc

     #define DEVICEID                .132
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x100
#endif

#ifdef __16F685
     #include p16F685.inc

     #define DEVICEID                .37
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F687
     #include p16F687.inc

     #define DEVICEID                .153
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F688
     #include p16F688.inc

     #define DEVICEID                .140
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F689
     #include p16F689.inc

     #define DEVICEID                .154
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F690
     #include p16F690.inc

     #define DEVICEID                .160
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F707
     #include p16F707.inc

     #define DEVICEID                .214
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F716
     #include p16F716.inc

     #define DEVICEID                .138
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x800
     #define END_GPR                 0x100
#endif

#ifdef __16F720
     #include p16F720.inc

     #define DEVICEID                .224
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F721
     #include p16F721.inc

     #define DEVICEID                .225
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F722
     #include p16F722.inc

     #define DEVICEID                .196
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F722A
     #include p16F722A.inc

     #define DEVICEID                .217
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F723
     #include p16F723.inc

     #define DEVICEID                .195
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F723A
     #include p16F723A.inc

     #define DEVICEID                .216
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F724
     #include p16F724.inc

     #define DEVICEID                .194
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F726
     #include p16F726.inc

     #define DEVICEID                .193
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F727
     #include p16F727.inc

     #define DEVICEID                .192
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F737
     #include p16F737.inc

     #define DEVICEID                .93
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F747
     #include p16F747.inc

     #define DEVICEID                .95
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F767
     #include p16F767.inc

     #define DEVICEID                .117
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F777
     #include p16F777.inc

     #define DEVICEID                .111
     #define WRITE_FLASH_BLOCKSIZE   .2
     #define ERASE_FLASH_BLOCKSIZE   .1024
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F785
     #include p16F785.inc

     #define DEVICEID                .144
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F818
     #include p16F818.inc

     #define DEVICEID                .38
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x400
     #define END_GPR                 0x200
#endif

#ifdef __16F819
     #include p16F819.inc

     #define DEVICEID                .39
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F870
     #include p16F870.inc

     #define DEVICEID                .104
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F871
     #include p16F871.inc

     #define DEVICEID                .105
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F872
     #include p16F872.inc

     #define DEVICEID                .71
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F873
     #include p16F873.inc

     #define DEVICEID                .75
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F873A
     #include p16F873A.inc

     #define DEVICEID                .114
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .8
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F874
     #include p16F874.inc

     #define DEVICEID                .73
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F874A
     #include p16F874A.inc

     #define DEVICEID                .115
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .8
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F876
     #include p16F876.inc

     #define DEVICEID                .79
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F876A
     #include p16F876A.inc

     #define DEVICEID                .112
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .8
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F877
     #include p16F877.inc

     #define DEVICEID                .77
     #define WRITE_FLASH_BLOCKSIZE   .1
     #define ERASE_FLASH_BLOCKSIZE   .1
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F877A
     #include p16F877A.inc

     #define DEVICEID                .113
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .8
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F882
     #include p16F882.inc

     #define DEVICEID                .256
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16F883
     #include p16F883.inc

     #define DEVICEID                .257
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F884
     #include p16F884.inc

     #define DEVICEID                .258
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F886
     #include p16F886.inc

     #define DEVICEID                .259
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F887
     #include p16F887.inc

     #define DEVICEID                .260
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F913
     #include p16F913.inc

     #define DEVICEID                .159
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F914
     #include p16F914.inc

     #define DEVICEID                .158
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16F916
     #include p16F916.inc

     #define DEVICEID                .157
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F917
     #include p16F917.inc

     #define DEVICEID                .156
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F946
     #include p16F946.inc

     #define DEVICEID                .163
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16F1507
     #include p16F1507.inc

     #define DEVICEID                .360
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16F1516
     #include p16F1516.inc

     #define DEVICEID                .180
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1517
     #include p16F1517.inc

     #define DEVICEID                .181
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1518
     #include p16F1518.inc

     #define DEVICEID                .182
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1519
     #include p16F1519.inc

     #define DEVICEID                .183
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1526
     #include p16F1526.inc

     #define DEVICEID                .172
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1527
     #include p16F1527.inc

     #define DEVICEID                .173
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1782
     #include p16F1782.inc

     #define DEVICEID                .336
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16F1783
     #include p16F1783.inc

     #define DEVICEID                .337
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1823
     #include p16F1823.inc

     #define DEVICEID                .313
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16F1824
     #include p16F1824.inc

     #define DEVICEID                .314
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1825
     #include p16F1825.inc

     #define DEVICEID                .315
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1826
     #include p16F1826.inc

     #define DEVICEID                .316
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16F1827
     #include p16F1827.inc

     #define DEVICEID                .317
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1828
     #include p16F1828.inc

     #define DEVICEID                .318
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1829
     #include p16F1829.inc

     #define DEVICEID                .319
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1847
     #include p16F1847.inc

     #define DEVICEID                .164
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1933
     #include p16F1933.inc

     #define DEVICEID                .280
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1934
     #include p16F1934.inc

     #define DEVICEID                .282
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1936
     #include p16F1936.inc

     #define DEVICEID                .283
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1937
     #include p16F1937.inc

     #define DEVICEID                .284
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1938
     #include p16F1938.inc

     #define DEVICEID                .285
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1939
     #include p16F1939.inc

     #define DEVICEID                .286
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1946
     #include p16F1946.inc

     #define DEVICEID                .296
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16F1947
     #include p16F1947.inc

     #define DEVICEID                .297
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

; PIC16HV610 does not support self-programming.
; PIC16HV616 does not support self-programming.
#ifdef __16HV785
     #include p16HV785.inc

     #define DEVICEID                .145
     #define WRITE_FLASH_BLOCKSIZE   .4
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16LF707
     #include p16LF707.inc

     #define DEVICEID                .215
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16LF720
     #include p16LF720.inc

     #define DEVICEID                .226
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16LF721
     #include p16LF721.inc

     #define DEVICEID                .227
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16LF722
     #include p16LF722.inc

     #define DEVICEID                .204
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16LF722A
     #include p16LF722A.inc

     #define DEVICEID                .219
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x200
#endif

#ifdef __16LF723
     #include p16LF723.inc

     #define DEVICEID                .203
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16LF723A
     #include p16LF723A.inc

     #define DEVICEID                .218
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16LF724
     #include p16LF724.inc

     #define DEVICEID                .202
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x200
#endif

#ifdef __16LF726
     #include p16LF726.inc

     #define DEVICEID                .201
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16LF727
     #include p16LF727.inc

     #define DEVICEID                .200
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x200
#endif

#ifdef __16LF1507
     #include p16LF1507.inc

     #define DEVICEID                .360
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1516
     #include p16LF1516.inc

     #define DEVICEID                .188
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1517
     #include p16LF1517.inc

     #define DEVICEID                .189
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1518
     #include p16LF1518.inc

     #define DEVICEID                .190
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1519
     #include p16LF1519.inc

     #define DEVICEID                .191
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1526
     #include p16LF1526.inc

     #define DEVICEID                .174
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1527
     #include p16LF1527.inc

     #define DEVICEID                .175
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1782
     #include p16LF1782.inc

     #define DEVICEID                .341
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1783
     #include p16LF1783.inc

     #define DEVICEID                .342
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1823
     #include p16LF1823.inc

     #define DEVICEID                .321
     #define WRITE_FLASH_BLOCKSIZE   .16
     #define ERASE_FLASH_BLOCKSIZE   .16
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1824
     #include p16LF1824.inc

     #define DEVICEID                .322
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1825
     #include p16LF1825.inc

     #define DEVICEID                .323
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1826
     #include p16LF1826.inc

     #define DEVICEID                .324
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1827
     #include p16LF1827.inc

     #define DEVICEID                .325
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1828
     #include p16LF1828.inc

     #define DEVICEID                .326
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1829
     #include p16LF1829.inc

     #define DEVICEID                .327
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1847
     #include p16LF1847.inc

     #define DEVICEID                .165
     #define WRITE_FLASH_BLOCKSIZE   .32
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1902
     #include p16LF1902.inc

     #define DEVICEID                .353
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x800
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1903
     #include p16LF1903.inc

     #define DEVICEID                .352
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1904
     #include p16LF1904.inc

     #define DEVICEID                .356
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1906
     #include p16LF1906.inc

     #define DEVICEID                .355
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1907
     #include p16LF1907.inc

     #define DEVICEID                .354
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1933
     #include p16LF1933.inc

     #define DEVICEID                .288
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1934
     #include p16LF1934.inc

     #define DEVICEID                .290
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x1000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1936
     #include p16LF1936.inc

     #define DEVICEID                .291
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1937
     #include p16LF1937.inc

     #define DEVICEID                .292
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1938
     #include p16LF1938.inc

     #define DEVICEID                .293
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1939
     #include p16LF1939.inc

     #define DEVICEID                .294
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x4000
     #define END_GPR                 0x1000
#endif

#ifdef __16LF1946
     #include p16LF1946.inc

     #define DEVICEID                .300
     #define WRITE_FLASH_BLOCKSIZE   .8
     #define ERASE_FLASH_BLOCKSIZE   .32
     #define END_FLASH               0x2000
     #define END_GPR                 0x1000
#endif
