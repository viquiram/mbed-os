/**HEADER********************************************************************
* 
* Copyright (c) 2013 Freescale Semiconductor;
* All Rights Reserved
*
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName: usb_device_config.h$
* $Version : 
* $Date    : 
*
* Comments:
*
*   
*
*END************************************************************************/

#ifndef __usb_dev_config_h__
#define __usb_dev_config_h__

/* if KHCI device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_KHCI                   1

/* if EHCI device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_EHCI                   0

/* if HID device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_HID                    1

/* if PHDC device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_PHDC                   1

/* if AUDIO device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_AUDIO                  1

/* if CDC device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_CDC                    1

/* if RNDIS  supported 
 * 1 supported
 * 0 not supported
 */
#if USBCFG_DEV_CDC
#define USBCFG_DEV_RNDIS_SUPPORT          1
#endif

/* if MSC device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_MSC		              1

/* if composite device supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_COMPOSITE              0

/* if device is self powered 
 * 1 self power
 * 0 bus power
 */
#define USBCFG_DEV_SELF_POWER             1

/* if device remote wakeup supported 
 * 1 supported
 * 0 not supported
 */
#define USBCFG_DEV_REMOTE_WAKEUP          0

/* how many device instance supported */
#define USBCFG_DEV_NUM                    2

/* how many endpoints are supported */
#define USBCFG_DEV_MAX_ENDPOINTS          (16) 

/* how many XDs are supported at most */
#define USBCFG_DEV_MAX_XDS                (32)

/* how many instance should be supported for one class type device */
#define USBCFG_DEV_MAX_CLASS_OBJECT       (1)

/* 
** Allow workaround for bug in the peripheral when unaligned buffer @4B address is used
** MGCT: <category name="USB DMA alignment fix">
*/
#define USBCFG_4BYTE_ALIGN_FIX 		(1)

/*
** The aligned buffer size for IN transactions, active when USBCFG_4BYTE_ALIGN_FIX is defined
  ** MGCT: <option type="number"/>
*/
#define USBCFG_KHCI_SWAP_BUF_MAX		(1024)

#ifdef _DEBUG
#undef _DEBUG
#endif
#endif
