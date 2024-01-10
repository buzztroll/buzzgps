SUBDIRS = src examples tests


if ENABLE_COVERAGE

cov-reset:
	@rm -fr coverage
	@find . -name "*.gcda" -exec rm {} \;
	@lcov --directory . --zerocounters

cov-report:
	@mkdir -p coverage
	@lcov --compat-libtool --directory . --capture --output-file coverage/app.info  
	@lcov --list coverage/app.info

cov-html:
	@make cov-report
	@genhtml -o coverage/ coverage/app.info 

cov:
	@make cov-html

clean-local:
	@make cov-reset 

check:
	@make cov
endif    
