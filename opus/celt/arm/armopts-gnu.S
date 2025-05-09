    .syntax unified
/* Copyright (C) 2013 Mozilla Corporation */
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES@ LOSS OF USE, @ DATA, OR
   PROFITS@ OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN  .if ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

@ Set the following to 1 if we have EDSP instructions
@  (LDRD/STRD, etc., ARMv5E and later).
 .set OPUS_ARM_MAY_HAVE_EDSP, 1

@ Set the following to 1 if we have ARMv6 media instructions.
 .set OPUS_ARM_MAY_HAVE_MEDIA, 0

@ Set the following to 1 if we have NEON (some ARMv7)
 .set OPUS_ARM_MAY_HAVE_NEON, 0

@ END:
