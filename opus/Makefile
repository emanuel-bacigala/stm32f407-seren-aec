
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

#vpath %.c celt
#vpath %.c celt/arm
#vpath %.c silk
#vpath %.c silk/arm
#vpath %.c silk/fixed
##vpath %.c silk/fixed/arm
##vpath %.c silk/float
#vpath %.c src
#vpath %.c include

CFLAGS = -O2 -Wall -std=gnu99 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fdata-sections -ffunction-sections
CFLAGS += -DHAVE_CONFIG_H -Icelt -Isilk -Isilk/fixed -Isilk/float -Isrc -Iinclude -I.

DIR=.
SRCS =	$(DIR)/src/mapping_matrix.c \
		$(DIR)/src/repacketizer.c \
		$(DIR)/src/opus_projection_encoder.c \
		$(DIR)/src/analysis.c \
		$(DIR)/src/opus_projection_decoder.c \
		$(DIR)/src/opus_decoder.c \
		$(DIR)/src/mlp.c \
		$(DIR)/src/opus_multistream.c \
		$(DIR)/src/opus_encoder.c \
		$(DIR)/src/opus_multistream_decoder.c \
		$(DIR)/src/opus.c \
		$(DIR)/src/mlp_data.c \
		$(DIR)/src/opus_multistream_encoder.c \
		$(DIR)/silk/resampler_down2_3.c \
		$(DIR)/silk/stereo_decode_pred.c \
		$(DIR)/silk/process_NLSFs.c \
		$(DIR)/silk/init_encoder.c \
		$(DIR)/silk/resampler_private_AR2.c \
		$(DIR)/silk/stereo_find_predictor.c \
		$(DIR)/silk/biquad_alt.c \
		$(DIR)/silk/decoder_set_fs.c \
		$(DIR)/silk/PLC.c \
		$(DIR)/silk/decode_frame.c \
		$(DIR)/silk/resampler_down2.c \
		$(DIR)/silk/quant_LTP_gains.c \
		$(DIR)/silk/code_signs.c \
		$(DIR)/silk/VAD.c \
		$(DIR)/silk/HP_variable_cutoff.c \
		$(DIR)/silk/LPC_inv_pred_gain.c \
		$(DIR)/silk/decode_pulses.c \
		$(DIR)/silk/NLSF_stabilize.c \
		$(DIR)/silk/tables_gain.c \
		$(DIR)/silk/stereo_encode_pred.c \
		$(DIR)/silk/control_codec.c \
		$(DIR)/silk/ana_filt_bank_1.c \
		$(DIR)/silk/sort.c \
		$(DIR)/silk/NLSF_VQ.c \
		$(DIR)/silk/table_LSF_cos.c \
		$(DIR)/silk/enc_API.c \
		$(DIR)/silk/sum_sqr_shift.c \
		$(DIR)/silk/resampler_private_down_FIR.c \
		$(DIR)/silk/NLSF2A.c \
		$(DIR)/silk/resampler_rom.c \
		$(DIR)/silk/VQ_WMat_EC.c \
		$(DIR)/silk/init_decoder.c \
		$(DIR)/silk/dec_API.c \
		$(DIR)/silk/NLSF_decode.c \
		$(DIR)/silk/resampler.c \
		$(DIR)/silk/tables_NLSF_CB_WB.c \
		$(DIR)/silk/NSQ_del_dec.c \
		$(DIR)/silk/LP_variable_cutoff.c \
		$(DIR)/silk/NLSF_unpack.c \
		$(DIR)/silk/stereo_MS_to_LR.c \
		$(DIR)/silk/control_audio_bandwidth.c \
		$(DIR)/silk/control_SNR.c \
		$(DIR)/silk/NLSF_del_dec_quant.c \
		$(DIR)/silk/fixed/apply_sine_window_FIX.c \
		$(DIR)/silk/fixed/autocorr_FIX.c \
		$(DIR)/silk/fixed/burg_modified_FIX.c \
		$(DIR)/silk/fixed/corrMatrix_FIX.c \
		$(DIR)/silk/fixed/encode_frame_FIX.c \
		$(DIR)/silk/fixed/find_LPC_FIX.c \
		$(DIR)/silk/fixed/find_LTP_FIX.c \
		$(DIR)/silk/fixed/find_pitch_lags_FIX.c \
		$(DIR)/silk/fixed/find_pred_coefs_FIX.c \
		$(DIR)/silk/fixed/k2a_FIX.c \
		$(DIR)/silk/fixed/k2a_Q16_FIX.c \
		$(DIR)/silk/fixed/LTP_analysis_filter_FIX.c \
		$(DIR)/silk/fixed/LTP_scale_ctrl_FIX.c \
		$(DIR)/silk/fixed/noise_shape_analysis_FIX.c \
		$(DIR)/silk/fixed/pitch_analysis_core_FIX.c \
		$(DIR)/silk/fixed/process_gains_FIX.c \
		$(DIR)/silk/fixed/regularize_correlations_FIX.c \
		$(DIR)/silk/fixed/residual_energy16_FIX.c \
		$(DIR)/silk/fixed/residual_energy_FIX.c \
		$(DIR)/silk/fixed/schur64_FIX.c \
		$(DIR)/silk/fixed/schur_FIX.c \
		$(DIR)/silk/fixed/vector_ops_FIX.c \
		$(DIR)/silk/fixed/warped_autocorrelation_FIX.c \
		$(DIR)/silk/resampler_private_IIR_FIR.c \
		$(DIR)/silk/A2NLSF.c \
		$(DIR)/silk/encode_pulses.c \
		$(DIR)/silk/inner_prod_aligned.c \
		$(DIR)/silk/sigm_Q15.c \
		$(DIR)/silk/log2lin.c \
		$(DIR)/silk/arm/arm_silk_map.c \
		$(DIR)/silk/interpolate.c \
		$(DIR)/silk/bwexpander.c \
		$(DIR)/silk/decode_indices.c \
		$(DIR)/silk/bwexpander_32.c \
		$(DIR)/silk/gain_quant.c \
		$(DIR)/silk/decode_parameters.c \
		$(DIR)/silk/CNG.c \
		$(DIR)/silk/NLSF_VQ_weights_laroia.c \
		$(DIR)/silk/NLSF_encode.c \
		$(DIR)/silk/NSQ.c \
		$(DIR)/silk/lin2log.c \
		$(DIR)/silk/tables_pulses_per_block.c \
		$(DIR)/silk/stereo_LR_to_MS.c \
		$(DIR)/silk/tables_NLSF_CB_NB_MB.c \
		$(DIR)/silk/decode_pitch.c \
		$(DIR)/silk/debug.c \
		$(DIR)/silk/tables_pitch_lag.c \
		$(DIR)/silk/LPC_analysis_filter.c \
		$(DIR)/silk/LPC_fit.c \
		$(DIR)/silk/pitch_est_tables.c \
		$(DIR)/silk/encode_indices.c \
		$(DIR)/silk/stereo_quant_pred.c \
		$(DIR)/silk/decode_core.c \
		$(DIR)/silk/shell_coder.c \
		$(DIR)/silk/tables_other.c \
		$(DIR)/silk/resampler_private_up2_HQ.c \
		$(DIR)/silk/check_control_input.c \
		$(DIR)/silk/tables_LTP.c \
		$(DIR)/celt/mathops.c \
		$(DIR)/celt/mdct.c \
		$(DIR)/celt/rate.c \
		$(DIR)/celt/entcode.c \
		$(DIR)/celt/kiss_fft.c \
		$(DIR)/celt/laplace.c \
		$(DIR)/celt/cwrs.c \
		$(DIR)/celt/celt_decoder.c \
		$(DIR)/celt/celt.c \
		$(DIR)/celt/entenc.c \
		$(DIR)/celt/bands.c \
		$(DIR)/celt/celt_lpc.c \
		$(DIR)/celt/vq.c \
		$(DIR)/celt/celt_encoder.c \
		$(DIR)/celt/modes.c \
		$(DIR)/celt/pitch.c \
		$(DIR)/celt/arm/armcpu.c \
		$(DIR)/celt/arm/arm_celt_map.c \
		$(DIR)/celt/entdec.c \
		$(DIR)/celt/quant_bands.c



OBJS = $(SRCS:.c=.o)

.PHONY: libopus.a

all: libopus.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libopus.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS)
