.PHONY: build build-service build-calibrate build-arm build-amd64 clean lint test run dev-build build-native

BINARY_NAME=motion-service
CAL_BINARY_NAME=motion-calibrate
BUILD_DIR=bin
GIT_REVISION=$(shell git rev-parse --short HEAD 2>/dev/null || echo "unknown")
GIT_DIRTY=$(shell git diff --quiet || echo "-dirty")
VERSION_FLAGS=-X main.gitRevision=$(GIT_REVISION)$(GIT_DIRTY) -X main.buildTime=$(shell date -u +%Y%m%d-%H%M%S)
LDFLAGS=-ldflags "-w -s -extldflags '-static' $(VERSION_FLAGS)"
CMD_DIR=cmd/motion-service
CAL_CMD_DIR=cmd/motion-calibrate

build: build-service build-calibrate

build-service:
	mkdir -p $(BUILD_DIR)
	CGO_ENABLED=0 GOOS=linux GOARCH=arm GOARM=7 go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME) ./$(CMD_DIR)

build-calibrate:
	mkdir -p $(BUILD_DIR)
	CGO_ENABLED=0 GOOS=linux GOARCH=arm GOARM=7 go build $(LDFLAGS) -o $(BUILD_DIR)/$(CAL_BINARY_NAME) ./$(CAL_CMD_DIR)

clean:
	rm -rf $(BUILD_DIR)

build-arm: build

build-amd64:
	mkdir -p $(BUILD_DIR)
	CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME)-amd64 ./$(CMD_DIR)
	CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build $(LDFLAGS) -o $(BUILD_DIR)/$(CAL_BINARY_NAME)-amd64 ./$(CAL_CMD_DIR)

lint:
	golangci-lint run

test:
	go test -v ./...

run:
	go run ./$(CMD_DIR)

dev-build:
	mkdir -p $(BUILD_DIR)
	go build -o $(BUILD_DIR)/$(BINARY_NAME) ./$(CMD_DIR)
	go build -o $(BUILD_DIR)/$(CAL_BINARY_NAME) ./$(CAL_CMD_DIR)

build-native:
	mkdir -p $(BUILD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(BINARY_NAME) ./$(CMD_DIR)
	go build $(LDFLAGS) -o $(BUILD_DIR)/$(CAL_BINARY_NAME) ./$(CAL_CMD_DIR)
