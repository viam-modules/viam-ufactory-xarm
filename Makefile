BIN_OUTPUT_PATH = bin
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
UNAME_S ?= $(shell uname -s)
UNAME_M ?= $(shell uname -m)

build:
	rm -rf bin
	go build -o $(BIN_OUTPUT_PATH)/viam-xarm

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz $(BIN_OUTPUT_PATH)/viam-xarm meta.json

clean:
	rm -rf $(BIN_OUTPUT_PATH)/viam-xarm $(BIN_OUTPUT_PATH)/module.tar.gz

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install \
		github.com/edaniels/golinters/cmd/combined \
		github.com/golangci/golangci-lint/cmd/golangci-lint \
		github.com/rhysd/actionlint/cmd/actionlint

gofmt:
	gofmt -w -s .

lint: export PATH := $(PWD)/$(TOOL_BIN):$(PATH)
lint: tool-install
	go mod tidy
	golangci-lint run

update-rdk:
	go get go.viam.com/rdk@latest
	go mod tidy

test: tool-install
	go test -v -race -failfast ./...
