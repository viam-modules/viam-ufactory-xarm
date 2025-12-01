BIN_OUTPUT_PATH = bin
TOOL_BIN = bin/gotools/$(shell uname -s)-$(shell uname -m)
PATH_WITH_TOOLS="`pwd`/$(TOOL_BIN):${PATH}"
UNAME_S ?= $(shell uname -s)
UNAME_M ?= $(shell uname -m)

build:
	rm -rf bin
	go build -o $(BIN_OUTPUT_PATH)/viam-xarm

module: build
	rm -f $(BIN_OUTPUT_PATH)/module.tar.gz
	tar czf $(BIN_OUTPUT_PATH)/module.tar.gz $(BIN_OUTPUT_PATH)/viam-xarm meta.json arm/3d_models

clean:
	rm -rf $(BIN_OUTPUT_PATH)/viam-xarm $(BIN_OUTPUT_PATH)/module.tar.gz

tool-install:
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/edaniels/golinters/cmd/combined@v0.0.5-0.20220906153528-641155550742
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/golangci/golangci-lint/cmd/golangci-lint@v1.64.8
	GOBIN=`pwd`/$(TOOL_BIN) go install github.com/rhysd/actionlint/cmd/actionlint@v1.7.8


gofmt:
	gofmt -w -s .

lint: gofmt tool-install
	go mod tidy
	PATH=$(PATH_WITH_TOOLS) golangci-lint run -c etc/.golangci.yaml --fix

update-rdk:
	go get go.viam.com/rdk@latest
	go mod tidy

test: tool-install
	go test -v -race -failfast ./...
