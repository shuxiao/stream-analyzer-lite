CXX ?= g++
APP_SLUG := stream-analyzer-lite
APP_VERSION := 0.1.0
LINUX_ARCH := amd64
RPM_ARCH := x86_64
ICON_DIR := ui/icons
ICON_SOURCE := $(ICON_DIR)/icon.svg
ICON_PNG_SIZES := 16,22,24,32,48,64,128,256,512

.PHONY: analyzer icons desktop-build desktop-dev normalize-linux-bundle-names clean

analyzer:
	$(MAKE) -C analyzer-core stream-analyzer-core

icons: $(ICON_SOURCE)
	mkdir -p $(ICON_DIR)
	cargo tauri icon $(ICON_SOURCE) -o $(ICON_DIR) --png $(ICON_PNG_SIZES)
	cp -f $(ICON_DIR)/256x256.png $(ICON_DIR)/128x128@2x.png
	cp -f $(ICON_DIR)/512x512.png $(ICON_DIR)/icon.png

desktop-build: icons analyzer
	cargo tauri build --bundles deb,rpm
	$(MAKE) normalize-linux-bundle-names

normalize-linux-bundle-names:
	if [ -f "target/release/bundle/deb/Stream Analyzer Lite_$(APP_VERSION)_$(LINUX_ARCH).deb" ]; then \
		mv -f "target/release/bundle/deb/Stream Analyzer Lite_$(APP_VERSION)_$(LINUX_ARCH).deb" \
			"target/release/bundle/deb/$(APP_SLUG)_$(APP_VERSION)_$(LINUX_ARCH).deb"; \
	fi
	if [ -f "target/release/bundle/rpm/Stream Analyzer Lite-$(APP_VERSION)-1.$(RPM_ARCH).rpm" ]; then \
		mv -f "target/release/bundle/rpm/Stream Analyzer Lite-$(APP_VERSION)-1.$(RPM_ARCH).rpm" \
			"target/release/bundle/rpm/$(APP_SLUG)-$(APP_VERSION)-1.$(RPM_ARCH).rpm"; \
	fi

desktop-dev: icons analyzer
	cargo tauri dev

clean:
	$(MAKE) -C analyzer-core clean
	cargo clean
