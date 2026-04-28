// SPDX-FileCopyrightText: © 2023 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only

#include "interrupt.h"

#include <linux/pci.h>
#include <linux/types.h>
#include <linux/interrupt.h>

#include "device.h"
#include "enumerate.h"
#include "blackhole.h"

static irqreturn_t irq_handler(int irq, void *device)
{
	struct tenstorrent_device *tt_dev = device;
	struct blackhole_device *bh;

	// Check if this is a Blackhole device with tt_pcie_log enabled
	if (tt_dev->dev_class->name &&
	    strcmp(tt_dev->dev_class->name, "Blackhole") == 0) {
		bh = tt_dev_to_bh_dev(tt_dev);
		if (bh->tt_pcie_log_enabled && bh->log_buffer_virt) {
			// Schedule log processing work
			schedule_work(&bh->log_work);
		}
	}

	return IRQ_HANDLED;
}

bool tenstorrent_enable_interrupts(struct tenstorrent_device *tt_dev)
{
	if (pci_alloc_irq_vectors(tt_dev->pdev, 1, 1, PCI_IRQ_ALL_TYPES) <= 0)
		goto out_pci_alloc_irq_vectors_failed;

	if (request_irq(pci_irq_vector(tt_dev->pdev, 0), irq_handler,
			IRQF_SHARED, TENSTORRENT, tt_dev) != 0)
		goto out_request_irq_failed;

	tt_dev->interrupt_enabled = true;
	return true;

out_request_irq_failed:
	pci_free_irq_vectors(tt_dev->pdev);
out_pci_alloc_irq_vectors_failed:
	return false;
}

void tenstorrent_disable_interrupts(struct tenstorrent_device *tt_dev)
{
	if (tt_dev->interrupt_enabled) {
		free_irq(pci_irq_vector(tt_dev->pdev, 0), tt_dev);
		pci_free_irq_vectors(tt_dev->pdev);
		tt_dev->interrupt_enabled = false;
	}
}
